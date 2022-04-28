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
#include "communication/PC2D.h"
#include "queue.h"
#include "communication/D2PC.h"

bool demo_done;

/*
 * @Author: Hanyuan Ban
 * @Param lq The receiving local queue, q The queue handling serial data.
 */
void receive_message(Queue* lq, Queue* q) {
	for (int i = 0; i < q->count; i++) {
		enqueue(lq, dequeue(q));
	}
}

/*
 * @Author: Hanyuan Ban
 * @Param mes The message needed to be filled, q The receiving local queue, len Size of message
 * @Return A flag indicates "parse failure" (0), "parse successful" (1), "parsed more than one message" (No. of message).
 */
int parse_message(PC2D_message_p mes, Queue* q, uint8_t len) {
	uint16_t header_p = q->first;
	uint16_t checksum_p = q->first;
	uint16_t pointer = q->first;
	uint8_t val;
	int iter = q->count;
	int result = 0;
	for (int j = 0; j < iter; j++) {
		val = q->items[pointer];	// read the data
		// if it is the header
		if ((char)val == '*') {
			header_p = pointer; // set the header pointer to here
			// remove the unparsed bytes
			for (int i = q->first; i < header_p; i++) {
				dequeue(q);
			}
			pointer = q->first;
		}
		// if it is a checksum
		if (val == len) {
			checksum_p = pointer; // set the checksum pointer to here
			// if exactly a message length
			if (checksum_p - header_p == len - 1) {
				memcpy(&mes, &(q->items[header_p]), len);	// copy the data to message
				for(int i = 0; i < len; i++) {
					dequeue(q); // delete the data from queue
				}
				pointer = q->first;
				result++;
			}
		}
		pointer++;
	}
	return result;
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

Queue local_receive_q;
PC2D_message rec_mes;

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

	init_queue(&local_receive_q);

	uint32_t counter = 0;
	demo_done = false;
	wireless_mode = false;

	rec_mes = create_message();
	int parse_result = 0;

	while (!demo_done) {
		if (rx_queue.count) {
			receive_message(&local_receive_q, &rx_queue);
		}

		parse_result = parse_message(&rec_mes, &local_receive_q, sizeof(rec_mes));
		if (parse_result >= 1) {
			printf("Mode: %d\n", rec_mes.mode);
			printf("Control: %d %d %d\n", rec_mes.control.x, rec_mes.control.y, rec_mes.control.z);
			if (parse_result > 1) {
				printf("Over-parsed %d messages", parse_result - 1);
			}
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
