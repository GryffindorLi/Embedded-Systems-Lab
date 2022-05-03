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
// #include "PC2D.h"

uint16_t motor[4];
int16_t ae[4];
bool wireless_mode;

// for IMU:
int16_t phi, theta, psi;
int16_t sp, sq, sr;
int16_t sax, say, saz;

// angle definitions:
int16_t yaw, pitch, roll;

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

set control gains:
Kpy = 57.9; Kiy = 3.35; Kdy = 250.0; // yaw
Kpp = 1.48; Kip = 0.0856; Kdp = 6.39; // pitch
Kpr = 1.48; Kir = 0.0856; Kdr = 6.39; // roll

// filter settings:
dt = 0.1; // in seconds
gyro_rate = 0.98;
acc_rate = 0.02;

// control variables:
float error[3];
float prev_error[3];
float derror[3];
float ierror[3];

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
	// combine gyro and accelerometer to remove drift:
	pitch = gyro_rate*(phi+sp*dt) + acc_rate*sax;
	roll = gyro_rate*(theta+sq*dt) + acc_rate*say;
	yaw = gyro_rate *(psi+sr*dt) + acc_rate*saz;
}

void get_error(pc_msg *mes){
	// convert degree to radian = pi/180 = 0.0174533
	// IMU angles to radians = 1/10430

	// find the error between control input and filtered IMU values:
	error[1] = mes->cm.control.yaw*0.0174533 - yaw/10430;
	error[2] = mes->cm.control.pitch*0.0174533 - pitch/10430;
	error[3] = mes->cm.control.roll*0.0174533 - roll/10430;

	// compute the derivative of the error:
	derror[1] = (error[1] - prev_error[1])/dt;
	derror[2] = (error[2] - prev_error[2])/dt;
	derror[3] = (error[3] - prev_error[3])/dt;

	// compute the integral of the error:
	ierror[1] = ((error[1] + prev_error[1])/2)*dt;
	ierror[2] = ((error[2] + prev_error[2])/2)*dt;
	ierror[3] = ((error[2] + prev_error[2])/2)*dt;
}

void controller(pc_msg *mes){
	// define all 3 PID controllers
	yaw_command = Kpy*error[1] + Kiy*ierror[1] + Kdy*derror[1];
	pitch_command = Kpp*error[2] + Kip*ierror[2] + Kdp*derror[2];
	roll_command = Kpr*error[3] + Kir*ierror[3] + Kdr*derror[3];

	// calculate motor outputs, scale them between 0 and 800:
	ae[0] = min(800, max(0, mes->cm.control.throttle - yaw_command + pitch_command + roll_command)); 
	ae[1] = min(800, max(0, mes->cm.control.throttle + yaw_command + pitch_command - roll_command)); 
	ae[2] = min(800, max(0, mes->cm.control.throttle + yaw_command - pitch_command + roll_command)); 
	ae[3] = min(800, max(0, mes->cm.control.throttle - yaw_command - pitch_command - roll_command)); 
}

// ------------- Manual Mode only ----------------
void controller_manual(pc_msg *mes){
	ae[0] = min(800, max(0, mes->cm.control.throttle - mes->cm.control.yaw + mes->cm.control.pitch + mes->cm.control.roll)); 
	ae[1] = min(800, max(0, mes->cm.control.throttle + mes->cm.control.yaw + mes->cm.control.pitch - mes->cm.control.roll)); 
	ae[2] = min(800, max(0, mes->cm.control.throttle + mes->cm.control.yaw - mes->cm.control.pitch + mes->cm.control.roll)); 
	ae[3] = min(800, max(0, mes->cm.control.throttle - mes->cm.control.yaw - mes->cm.control.pitch - mes->cm.control.roll)); 
}
---------------------------------------------------
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