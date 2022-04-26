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

/*------------------------------------------------------------
 * parse message
 *------------------------------------------------------------
 */

int receive_message(PC2D_message_p mes, Queue* q, int len) {
	for (int i = 0; i < len; i++) {
		uint8_t val = dequeue(q);
		if (val == (uint8_t) -1) {
			printf("wrong at %d\n", i);
			return -1;
		}
		if (i == 0) {
			mes->checksum = val;
			if (val != len) {
				printf("wrong checksum");
				return -1;
			}
		}
		if (i == 1) mes->mode = val;
		if (i == 2) mes->control.x = (uint16_t) val;
		if (i == 3) mes->control.x |= ((uint16_t) val) << 8;
		if (i == 4) mes->control.y = (uint16_t) val;
		if (i == 5) mes->control.y |= ((uint16_t) val) << 8;
		if (i == 6) mes->control.z = (uint16_t) val;
		if (i == 7) mes->control.z |= ((uint16_t) val) << 8;
		if (i == 8) mes->key = (char) val;
	}
	return 1;
}

/*------------------------------------------------------------------
 * process_key -- process command keys
 *------------------------------------------------------------------
 */
void process_key(uint8_t c)
{
	switch (c) {
	case 'q':
		ae[0] += 10;
		break;
	case 'a':
		ae[0] -= 10;
		if (ae[0] < 0) ae[0] = 0;
		break;
	case 'w':
		ae[1] += 10;
		break;
	case 's':
		ae[1] -= 10;
		if (ae[1] < 0) ae[1] = 0;
		break;
	case 'e':
		ae[2] += 10;
		break;
	case 'd':
		ae[2] -= 10;
		if (ae[2] < 0) ae[2] = 0;
		break;
	case 'r':
		ae[3] += 10;
		break;
	case 'f':
		ae[3] -= 10;
		if (ae[3] < 0) ae[3] = 0;
		break;
	case 27:
		demo_done = true;
		break;
	default:
		nrf_gpio_pin_toggle(RED);
	}
}


/*------------------------------------------------------------------
 * main -- everything you need is here :)
 *------------------------------------------------------------------
 */

/*
 * @Author Hanyuan Ban
 * @Author Zirui Li
 */
int main(void)
{
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
	demo_done = false;
	wireless_mode = false;

	PC2D_message rec_mes = create_message();

	while (!demo_done) {
		if (rx_queue.count) {
			if (receive_message(&rec_mes, &rx_queue, sizeof(rec_mes)) == 1) {
				printf("RECEIVED_MESSAGE:\n");
				printf("checksum: %d\n", rec_mes.checksum);
				printf("mode: %d\n", rec_mes.mode);
				printf("controls: %d %d %d\n", rec_mes.control.x, rec_mes.control.y, rec_mes.control.z);
				printf("key: %c\n", rec_mes.key);
			}
		}
		if (ble_rx_queue.count) {
			process_key(dequeue(&ble_rx_queue));
		}

		if (check_timer_flag()) {
			if (counter++%20 == 0) {
				nrf_gpio_pin_toggle(BLUE);
			}
			/*
			adc_request_sample();
			read_baro();

			printf("%10ld | ", get_time_us());
			printf("%3d %3d %3d %3d | ",ae[0], ae[1], ae[2], ae[3]);
			printf("%6d %6d %6d | ", phi, theta, psi);
			printf("%6d %6d %6d | ", sp, sq, sr);
			printf("%4d | %4ld | %6ld \n", bat_volt, temperature, pressure);
			*/

			D2PC_message m = init_message();
			bytes_array* b = to_bytes_array(&m);
			for (int i = 0; i < 10; ++i){
				uart_put(b->bytes[i]);
			}
			delete_message(&m);
			delete_bytes_array(b);

			clear_timer_flag();
		}

		if (check_sensor_int_flag()) {
			get_sensor_data();
			run_filters_and_control();
		}
	}	

	printf("\n\t Goodbye \n\n");
	nrf_delay_ms(100);

	NVIC_SystemReset();
}