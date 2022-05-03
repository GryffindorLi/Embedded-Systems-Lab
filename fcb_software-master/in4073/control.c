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
#include "communication/PC2D.h"

uint16_t motor[4];
int16_t ae[4];
bool wireless_mode;

// for IMU:
int16_t phi, theta, psi;
int16_t sp, sq, sr;
int16_t sax, say, saz;

// angle definitions:
int16_t yaw, pitch, roll;

// time settings:
int16_t dt = 100; // in milliseconds (ms)

// control variables:
int16_t error[3];
int16_t prev_error[3];
int16_t derror[3];
int16_t ierror[3];

int16_t yaw_command;
int16_t pitch_command;
int16_t roll_command;

// control input:
pc_msg msg;

void update_motors(void)
{
	motor[0] = ae[0];
	motor[1] = ae[1];
	motor[2] = ae[2];
	motor[3] = ae[3];
}

void filter_angles(void){
	// angles are in degrees
	// combine gyro and accelerometer to remove drift:
	// constants are LSB values from mpu6050.c

	float gyro_rate = 0.98;
	float acc_rate = 0.02;

	pitch = (int16_t) 182*(gyro_rate*(phi/182+(sp*dt)/16.4) + acc_rate*(sax/16384));
	roll = (int16_t) 182*(gyro_rate*(theta/182+(sq*dt)/16.4) + acc_rate*(say/16384));
	yaw = (int16_t) 182*(gyro_rate *(psi/182+(sr*dt)/16.4) + acc_rate*(saz/16384));
}

void get_error(pc_msg *mes){
	// convert degree to radian = pi/180 = 0.0174533
	// IMU angles to radians = 1/10430
	// TODO: check if int16_t can have negative values

	// find the error between control input and filtered IMU values:
	error[0] = (int16_t) mes->cm.control.yaw - yaw;
	error[1] = (int16_t) mes->cm.control.pitch - pitch;
	error[2] = (int16_t) mes->cm.control.roll - roll;

	// compute the derivative of the error:
	derror[0] = (int16_t) (error[0] - prev_error[0])/(dt/1000);
	derror[1] = (int16_t) (error[1] - prev_error[1])/(dt/1000);
	derror[2] = (int16_t) (error[2] - prev_error[2])/(dt/1000);

	// compute the integral of the error:
	ierror[0] = (int16_t) ((error[0] + prev_error[0])/2)*(dt/1000);
	ierror[1] = (int16_t) ((error[1] + prev_error[1])/2)*(dt/1000);
	ierror[2] = (int16_t) ((error[2] + prev_error[2])/2)*(dt/1000);
}

void controller(pc_msg *mes){
	// set control gains:
	float Kpy = 57.9; float Kiy = 3.35; float Kdy = 250.0; // yaw
	float Kpp = 1.48; float Kip = 0.0856; float Kdp = 6.39; // pitch
	float Kpr = 1.48; float Kir = 0.0856; float Kdr = 6.39; // roll

	// define all 3 PID controllers
	yaw_command = (int16_t) Kpy*error[0] + Kiy*ierror[0] + Kdy*derror[0];
	pitch_command = (int16_t) Kpp*error[1] + Kip*ierror[1] + Kdp*derror[1];
	roll_command = (int16_t) Kpr*error[2] + Kir*ierror[2] + Kdr*derror[2];

	// calculate motor outputs, scale them between 0 and 800:
	ae[0] = (mes->cm.control.throttle - yaw_command + pitch_command + roll_command)/82; 
	if( ae[0] < 0 ) {
		ae[0] = 0;
	}
	if( ae[0] > 800 ) {
		ae[0] = 800;
	}

	ae[1] = (mes->cm.control.throttle + yaw_command + pitch_command - roll_command)/82; 
	if( ae[1] < 0 ) {
		ae[1] = 0;
	}
	if( ae[1] > 800 ) {
		ae[1] = 800;
	}

	ae[2] = (mes->cm.control.throttle + yaw_command - pitch_command + roll_command)/82; 
	if( ae[2] < 0 ) {
		ae[2] = 0;
	}
	if( ae[2] > 800 ) {
		ae[2] = 800;
	}

	ae[3] = (mes->cm.control.throttle - yaw_command - pitch_command - roll_command)/82;
	if( ae[3] < 0 ) {
		ae[3] = 0;
	}
	if( ae[3] > 800 ) {
		ae[3] = 800;
	} 
}


void run_filters_and_control()
{
	// fancy stuff here
	// control loops and/or filters
	filter_angles();
	get_error(&msg);
	controller(&msg);

	// ae[0] = xxx, ae[1] = yyy etc etc
	update_motors();
}
