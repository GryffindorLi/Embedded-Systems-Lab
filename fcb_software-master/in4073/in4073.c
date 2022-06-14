/*------------------------------------------------------------------
 *  in4073.c -- test QR engines and sensors
 *
 *  reads ae[0-3] uart rx queue
 *  (q,w,e,r increment, a,s,d,f decrement)
 *
 *  prints timestamp, ae[0-3], sensors to uart tx queue
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  June 2016
 *------------------------------------------------------------------
 */
#include "in4073.h"
#include <stdbool.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "adc.h"
#include "barometer.h"
#include "gpio.h"
#include "spi_flash.h"
#include "timers.h"
#include "twi.h"
#include "hal/uart.h"
#include "control.h"
#include "mpu6050/mpu6050.h"
#include "utils/quad_ble.h"
#include "PC2D.h"
#include "queue.h"
#include "D2PC.h"
#include "D2PC_drone.h"
#include "keyboard.h"
#include "config.h"
#include "logs_flash.h"

bool demo_done;
uint32_t panic_to_safe_timer = -1;
int32_t yaw, pitch, roll;
int start_calibration;
int16_t* aes;
uint16_t height_control_throttle;
char *mode_str[8] = {"safe", "panic", "manual", "calibration", "yaw-control", "full-control", "raw-mode", "height-mode"};

int r_state = 0;
int c_state = 0;
int Md_flag = 0;
int Ct_flag = 0;
uint8_t Md_buffer[3] = {'M', 'd', -1};
uint8_t Ct_buffer[14] = {'C', 't', 0};
uint8_t Ct_p = 2;

CTRL_msg rec_msg;
uint8_t current_mode = 0;
controls current_control;
char current_key;

/*
 * @Author: Zirui Li
 */
bool check_battery(){
    if (bat_volt < BATTERY_LEVEL && current_mode != MODE_PANIC && current_mode != MODE_SAFE){
		return false;
	}
	return true;
}


/*
 * @Author: Hanyuan Ban
 * @Param lq The receiving local queue, q The queue handling serial data.
 */
void receive_message(Queue* q) {
	uint8_t c;
	for (int i = 0; i < q->count; i++) {
		c = dequeue(q);
		if (r_state == 0) {
			if (c == 'M') {	// mode header 1
				r_state = 1; // wait for mode header 2
			} else { // receive in control buffer
				if (c_state != 2) {  // if not recognized header
					if (c == 'C') c_state = 1;	// control header 1
					else if (c == 't') { //control header 2
						if (c_state == 1) c_state = 2;
						else c_state = 0;
					} else c_state = 0;
				} else {
					Ct_buffer[Ct_p++] = c;
					if (Ct_p == sizeof(CTRL_msg)) {
						c_state = 0;
						Ct_p = 2;
						if (Ct_buffer[sizeof(CTRL_msg) - 1] == sizeof(CTRL_msg)) {	//check sum
							Ct_flag = 1;
							memcpy(&rec_msg, Ct_buffer, sizeof(CTRL_msg));
						}
					}
				}
			}
		} else if (r_state == 1) { // mode header 2
			if (c == 'd') {
				r_state = 2;
			} else {
				r_state = 0;
				if (c_state == 2) {
					if (Ct_p != sizeof(CTRL_msg) - 1) Ct_buffer[Ct_p++] = 'M';  // bring the byte back
					else {  // not check sum
						c_state = 0;
						Ct_p = 2;
					}
				}
			}
		} else { // full mode message
			Md_flag = 1;
			Md_buffer[2] = c;
			r_state = 0;
			return;
		}
	}
}


uint8_t on_mode_change(uint8_t current_mode, int16_t* aes) {
	uint8_t mode = Md_buffer[2];
	switch (mode) {
		case MODE_SAFE:
			if (current_mode == MODE_SAFE)
				return current_mode;
			else {
				start_calibration = 0;
				printf("\n---===Motors still running! Going to PANIC mode!===---\n");
				return MODE_PANIC;
			}
			break;

		case MODE_PANIC:
			if (current_mode == MODE_SAFE) {
				return current_mode;
			} else if (current_mode == MODE_PANIC) {
				return current_mode;
			} else {
				start_calibration = 0;
				panic_to_safe_timer = get_time_us();
				printf("\n---===Entering PANIC mode!===---\n");
				return mode;
			}
			break;

		case MODE_MANUAL:
			if (current_mode == MODE_MANUAL) {
				return current_mode;
			} else {
				if (aes[0] + aes[1] + aes[2] + aes[3] != 0) {
					reset_control_offset();
					printf("\n---===Stop motors first to enter MANUAL mode!===---\n");
					return current_mode;
				} else {
					start_calibration = 0;
					printf("\n---===Entering MANUAL mode!===---\n");
					return mode;
				}
			}
			break;

		case MODE_CALIBRATION:
			if (current_mode == MODE_CALIBRATION) {
				return current_mode;
			} else {
				if (aes[0] + aes[1] + aes[2] + aes[3] != 0) {
					reset_control_offset();
					printf("\n---===Stop motors first to enter CALIBRATION mode!===---\n");
					return current_mode;
				} else {
					start_calibration = 1;
					printf("\n---===Entering CALIBRATION mode!===---\n");
					return mode;
				}
			}
			break;

		case MODE_YAW_CONTROL:
			if (current_mode == MODE_YAW_CONTROL) {
				return current_mode;
			} else {
				if (aes[0] + aes[1] + aes[2] + aes[3] != 0) {
					reset_control_offset();
					printf("\n---===Stop motors first to enter YAW CONTROL mode!===---\n");
					return current_mode;
				} else {
					start_calibration = 0;
					printf("\n---===Entering YAW CONTROL mode!===---\n");
					return mode;
				}
			}
			break;

		case MODE_FULL_CONTROL:
			if (current_mode == MODE_FULL_CONTROL) {
				return current_mode;
			} else {
				if ((current_mode == MODE_HEIGHT_CONTROL) && (aes[0] + aes[1] + aes[2] + aes[3] != 0)) {
					start_calibration = 0;
					height_control_throttle = current_control.throttle;
					printf("\n---===Entering FULL CONTROL mode!===---\n");
					return mode;
				} else if (aes[0] + aes[1] + aes[2] + aes[3] != 0){
					reset_control_offset();
					printf("\n---===Stop motors first to enter FULL CONTROL mode!===---\n");
					return current_mode;
				} else {
					start_calibration = 0;
					height_control_throttle = current_control.throttle;
					printf("\n---===Entering FULL CONTROL mode!===---\n");
					return mode;
				}
			}
			break;

		case MODE_RAW:
			if (current_mode == MODE_RAW) {
				return current_mode;
			} else {
				if (aes[0] + aes[1] + aes[2] + aes[3] != 0) {
					reset_control_offset();
					printf("\n---===Stop motors first to enter RAW mode!===---\n");
					return current_mode;
				} else {
					start_calibration = 0;
					printf("\n---===Entering RAW mode!===---\n");
					return mode;
				}
			}
			break;

		case MODE_HEIGHT_CONTROL:
			if (current_mode == MODE_HEIGHT_CONTROL)
				return current_mode;
			else if (current_mode == MODE_FULL_CONTROL) {
				start_calibration = 0;
				height_control_throttle = current_control.throttle;
				printf("\n---===Entering HEIGHT CONTROL mode!===---\n");
				return mode;
			} else {
				printf("\n---===Go to full control first to enter HEIGHT CONTROL mode!===---\n");
				return current_mode;
			}
			break;

		default:
			return current_mode;
	}
}

controls on_set_control(CTRL_msg* msg) {
	if (aes[0] / 4 + aes[1] / 4 + aes[2] / 4 + aes[3] / 4 < 125) {
		if (msg->control.roll != 0 || msg->control.pitch != 0 || msg->control.yaw != 0 ) {
			msg->control.roll = 0;
			msg->control.pitch = 0;
			msg->control.yaw = 0;
			//printf("\n Throttle up before Acrobats!\n");
		}
	}
	return msg->control;
}

char on_set_key(CTRL_msg* msg) {
	return msg->key;
}

void led_indicator(uint8_t current_mode) {
	// change lights, set clear is reversed due to hardware
	if (current_mode == MODE_PANIC) {
		nrf_gpio_pin_clear(RED);
		nrf_gpio_pin_set(YELLOW);
		nrf_gpio_pin_set(GREEN);
	} else if (current_mode == MODE_SAFE) {
		nrf_gpio_pin_set(RED);
		nrf_gpio_pin_clear(YELLOW);
		nrf_gpio_pin_set(GREEN);
	} else {
		nrf_gpio_pin_set(RED);
		nrf_gpio_pin_set(YELLOW);
		nrf_gpio_pin_clear(GREEN);
	}
}
/*------------------------------------------------------------------
 * main -- everything you need is here :)
 *------------------------------------------------------------------
 */

/*
 * @Author Hanyuan Ban
 * @Author Zirui Li
 * @Author Karan
 */

int main(void){
	// --------------------------------INITIALIZATION------------------------------------
	uart_init();
	gpio_init();
	timers_init();
	adc_init();
	twi_init();
	imu_init(true, 100);
	baro_init();
	spi_flash_init();
	quad_ble_init();
	// check ctrl frequency v
	uint32_t s_timer = get_time_us();
	uint32_t s_period = 0;
	uint32_t print_s_timer = get_time_us();
	// check ctrl frequency ^
	uint32_t counter = 0;
	uint32_t start_time = 0;
	uint32_t end_time = 0;
	uint32_t loop_time = 0;
	uint32_t idle_timer = get_time_us();
	demo_done = false;
	wireless_mode = false;
	calibration = 0;

	// --------------------------------MAIN LOOP------------------------------------

	while (!demo_done) {
		if (check_loop_time)
			start_time = get_time_us();

		if (rx_queue.count) {
			receive_message(&rx_queue);
		}
		// check if there is message
		if (Md_flag == 1) {
			current_mode = on_mode_change(current_mode, aes);
			Md_flag = 0;
			UART_watch_dog = 1000;
		}
		if (Ct_flag == 1) {
			current_control = on_set_control(&rec_msg);
			current_key = on_set_key(&rec_msg);
			Ct_flag = 0;
			UART_watch_dog = 1000;
			s_period = (get_time_us() - s_timer + 50) / 1000;
			s_timer = get_time_us();
			if (get_time_us() - print_s_timer > 1000000) {
				if (current_mode != MODE_CALIBRATION)
					printf("Loop time : %ld ms\n", s_period);
				print_s_timer = get_time_us();
			}
		}
		
		#ifdef check_battery
			if (!check_battery()){
				current_mode = MODE_PANIC;
				panic_to_safe_timer = get_time_us();
				printf("\nThe battery is too low!\n");
			}
		#endif

		// PANIC to SAFE
		if (panic_to_safe_timer != -1) {
			if (get_time_us() - panic_to_safe_timer > panic_to_safe_delay) {
				current_mode = MODE_SAFE;
				panic_to_safe_timer = -1;
				printf("\nentered SAFE MODE\n");
			}
		}

		// HEIGHT to FULL control
		if (current_mode == MODE_HEIGHT_CONTROL){
			if (calibration == 0){
				current_mode = MODE_FULL_CONTROL;
				printf("\nCALIBRATE first before HEIGHT CONTROL\n");
				printf("\nentered FULL CONTROL MODE\n");
			}
			else if ((current_control.throttle - height_control_throttle > 200) ||
		    		 (current_control.throttle - height_control_throttle < -200)){
				current_mode = MODE_FULL_CONTROL;
				printf("\nTHROTTLE disabled HEIGHT CONTROL\n");
				printf("\nentered FULL CONTROL MODE\n");
			}
			else if (current_key == '5'){
				current_mode = MODE_FULL_CONTROL;
				printf("\nKEY disbaled HEIGHT CONTROL\n");
				printf("\nentered FULL CONTROL MODE\n");
			}
		}

		// Change lights
		led_indicator(current_mode);

		// Check Disconnection
		if (get_time_us() - idle_timer > 1000) {
			UART_watch_dog -= 3;
			idle_timer = get_time_us();
			if (UART_watch_dog < 1) {
				if (current_mode > MODE_PANIC) current_mode = MODE_PANIC;
				panic_to_safe_timer = get_time_us();
				printf("\nDISCONNECTION!!\n");
			}
		}

		if (check_timer_flag()) {
			// 20HZ
			if (counter++%20 == 0) {
				// 1HZ
				nrf_gpio_pin_toggle(BLUE);
				if (current_mode != MODE_CALIBRATION){
					printf("\n--==<< controls (trpy): %d %d %d %d >>==--\n", current_control.throttle, current_control.roll,
																			 current_control.pitch, current_control.yaw);
					printf("\nMotor0: %d, Motor1: %d, Motor2: %d, Motor3: %d\n", aes[0], aes[1], aes[2], aes[3]);
					printf("theta: %d, sq: %d, -sax: %d, pitch: %ld\n", theta, sq, -sax, pitch);
					if (print_angles)
						printf("Yaw: %ld, Pitch: %ld, Roll: %ld\n", yaw, pitch, roll);
					if (PID_prints)
						printf("P: %d, I: %d, D: %d\n", p_roll, i_roll, d_roll);

					printf("\nMode: %s\n", mode_str[current_mode]);
					printf("Height throttle: %d\n", height_control_throttle);
				}

				// D2PC_message m = init_message();
				// send_data(&m);
			}

			adc_request_sample();
			read_baro();
			// D2PC_message p = init_message();
			// send_data(&m);

			// write_D2PC_msg_flash(&p);
#ifndef LOG_FROM_TERMINAL
			send_data(&m);
#endif

#ifdef LOG_FROM_TERMINAL
/*
			print_data(&m);
			printf("index: %hu,mode: %hu,battery: %hu,"\
            "yaw: %ld,pitch: %ld,roll: %ld,"\
            "filtered_yaw: %hd,filtered_pitch: %hd,filtered_roll: %hd,"\
            " motor1: %hd,motor2: %hd,motor3: %hd,motor4: %hd\n",
            p.idx, p.mode, p.battery, p.y, p.p, p.r,
            p.filtered_y, p.filtered_p, p.filtered_r, p.motor1,
            p.motor2, p.motor3, p.motor4);
*/
#endif

			clear_timer_flag();
		}


		if (check_sensor_int_flag()) {
			//100Hz
			get_sensor_data();

			aes = run_filters_and_control(current_control, current_key, current_mode);
			current_key = '\0';
		}

		if (check_loop_time) {
			end_time = get_time_us();
			loop_time = end_time - start_time;
			if (check_loop_time)
				printf("%ld\n", loop_time);
		}
	}

	printf("\n\t Goodbye \n\n");
	nrf_delay_ms(100);

	NVIC_SystemReset();
}
