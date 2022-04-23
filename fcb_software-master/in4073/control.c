/*------------------------------------------------------------------
 *  control.c -- here you can implement your control algorithm
 *		 and any motor clipping or whatever else
 *		 remember! motor input =  0-1000 : 125-250 us (OneShot125)
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  July 2016
 *------------------------------------------------------------------
 */

#include "control.h"
#include <inttypes.h>
#include <stdbool.h>
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "in4073.h"
#include "utils/quad_ble.h"
#include "utils/queue.h"
#include "mpu6050/mpu6050.h"
#include "uart.h"
#include "gpio.h"

uint16_t motor[4];
int16_t ae[4];
bool wireless_mode;

void update_motors(void)
{
	motor[0] = ae[0];
	motor[1] = ae[1];
	motor[2] = ae[2];
	motor[3] = ae[3];
}

void run_filters_and_control()
{

	// fancy stuff here
	// control loops and/or filters

	// ae[0] = xxx, ae[1] = yyy etc etc
	update_motors();
}
