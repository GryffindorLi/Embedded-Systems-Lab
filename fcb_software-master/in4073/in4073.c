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

bool demo_done;
uint32_t panic_to_safe_timer = -1;
int32_t yaw, pitch, roll;
int start_calibration;
int16_t* aes;

int r_state = 0;
int c_state = 0;
int Md_flag = 0;
int Ct_flag = 0;
uint8_t Md_buffer[3] = {'M', 'd', -1};
uint8_t Ct_buffer[12] = {'C', 't', 0};
uint8_t Ct_p = 2;

void send_data(D2PC_message_p m) {
    uart_put((uint8_t)(m->head));
	uart_put(m->mode);
	uart_put(m->battery);
	uart_put((uint8_t)(m->y >> 8));
	uart_put((uint8_t)(m->y & 0xff));
	uart_put((uint8_t)(m->p >> 8));
	uart_put((uint8_t)(m->p & 0xff));
	uart_put((uint8_t)(m->r>> 8));
	uart_put((uint8_t)(m->r & 0xff));
	uart_put((uint8_t)(m->filtered_y >> 8));
	uart_put((uint8_t)(m->filtered_y & 0xff));
	uart_put((uint8_t)(m->filtered_p >> 8));
	uart_put((uint8_t)(m->filtered_p & 0xff));
	uart_put((uint8_t)(m->filtered_r>> 8));
	uart_put((uint8_t)(m->filtered_r & 0xff));
	uart_put((uint8_t)(m->motor1 >> 8));
	uart_put((uint8_t)(m->motor1 & 0xff));
	uart_put((uint8_t)(m->motor2 >> 8));
	uart_put((uint8_t)(m->motor2 & 0xff));
	uart_put((uint8_t)(m->motor3 >> 8));
	uart_put((uint8_t)(m->motor3 & 0xff));
	uart_put((uint8_t)(m->motor4 >> 8));
	uart_put((uint8_t)(m->motor4 & 0xff));
	uart_put((uint8_t)(m->checksum >> 8));
	uart_put((uint8_t)(m->checksum & 0xff));
	uart_put((uint8_t)(m->tail));
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
							return;
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
			if (current_mode == MODE_SAFE) return current_mode;
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
					printf("\n---===Stop motors first to enter YAW CONTROL mode!===---\n");
					return current_mode;
				} else {
					start_calibration = 0;
					printf("\n---===Entering YAW CONTROL mode!===---\n");
					return mode;
				}
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
			printf("\n Throttle up before Acrobats!\n");
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


CTRL_msg rec_msg;
uint8_t current_mode = 0;
controls current_control;
char current_key;

int main(void)
{
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

	uint32_t counter = 0;
	uint32_t idle_timer = get_time_us();
	demo_done = false;
	wireless_mode = false;
	// uint32_t loop_monitor_start = 0;

	// --------------------------------MAIN LOOP------------------------------------

	while (!demo_done) {
		// receive message when there is message
		if (rx_queue.count) {
			receive_message(&rx_queue);
		}
		// check if there is message		
		if (Md_flag == 1) {
			current_mode = on_mode_change(current_mode, aes);
			Md_flag = 0;
		} 
		if (Ct_flag == 1) {
			memcpy(&rec_msg, Ct_buffer, sizeof(CTRL_msg));
			current_control = on_set_control(&rec_msg);
			current_key = on_set_key(&rec_msg);
			Ct_flag = 0;
		}

		// PANIC to SAFE
		if (panic_to_safe_timer != -1) {
			if (get_time_us() - panic_to_safe_timer > 500000) {
				current_mode = MODE_SAFE;
				panic_to_safe_timer = -1;
				printf("\nentered SAFE MODE\n");
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
				printf("\nDISCONNECTION!!\n");
			}
		}	

		if (check_timer_flag()) {
			// 20HZ
			if (counter++%20 == 0) {
				// 1HZ
				nrf_gpio_pin_toggle(BLUE);
				printf("\nMotor0: %d, Motor1: %d, Motor2: %d, Motor3: %d\n", aes[0], aes[1], aes[2], aes[3]);
			}

			adc_request_sample();
			read_baro();		

			clear_timer_flag();
		}

		if (check_sensor_int_flag()) {
			//100Hz
			get_sensor_data();
			aes = run_filters_and_control(current_control, current_key, current_mode);
		}
	}	

	printf("\n\t Goodbye \n\n");
	nrf_delay_ms(100);

	NVIC_SystemReset();
}

