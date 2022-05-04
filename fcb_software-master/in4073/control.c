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

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

uint16_t motor[4];
int16_t ae[4];
bool wireless_mode;

// for IMU:
int16_t phi, theta, psi;
int16_t sp, sq, sr;
int16_t sax, say, saz;

// angle definitions:
int32_t yaw, pitch, roll;

// control variables:
int32_t error[3];
int32_t prev_error[3];
int32_t derror[3];
int32_t ierror[3];

int32_t yaw_command;
int32_t pitch_command;
int32_t roll_command;

// motor range
int16_t min_motor = 10; // minimum motor value where it starts turning
int16_t manual_max_motor = 400; // max motor value in manual mode
int16_t max_motor = 1000; // max motor value in full control mode
int16_t safe_motor = 0; // safe mode motor value
int16_t panic_motor = 300; // panic mode motor value

// frequency control loop - 1/looptime
int8_t freq = 10; // hz

// control input:
pc_msg msg;

void update_motors(void)
{
	motor[0] = ae[0];
	motor[1] = ae[1];
	motor[2] = ae[2];
	motor[3] = ae[3];
}

// ------------- Manual Mode only ----------------
void controller_manual(pc_msg *msg){
	ae[0] = MIN(manual_max_motor, MAX(min_motor, msg->cm.control.throttle - msg->cm.control.yaw + msg->cm.control.pitch)); 
	ae[1] = MIN(manual_max_motor, MAX(min_motor, msg->cm.control.throttle + msg->cm.control.yaw - msg->cm.control.roll)); 
	ae[2] = MIN(manual_max_motor, MAX(min_motor, msg->cm.control.throttle - msg->cm.control.yaw - msg->cm.control.pitch)); 
	ae[3] = MIN(manual_max_motor, MAX(min_motor, msg->cm.control.throttle + msg->cm.control.yaw + msg->cm.control.roll)); 
}

// ------------- Control Mode only ----------------
void filter_angles(void){
	// angles are in degrees
	// combine gyro and accelerometer to remove drift:
	// constants are LSB values from mpu6050.c

	float gyro_rate = 0.98;
	float acc_rate = 0.02;
	float dt = 0.1;

	pitch = (int32_t) 182*(gyro_rate*(phi/182+(sp*dt)/16.4) + acc_rate*(sax/16384));
	roll = (int32_t) 182*(gyro_rate*(theta/182+(sq*dt)/16.4) + acc_rate*(say/16384));
	yaw = (int32_t) 182*(gyro_rate *(psi/182+(sr*dt)/16.4) + acc_rate*(saz/16384));
}

void get_error(pc_msg *mes){
	// convert degree to radian = pi/180 = 0.0174533
	// IMU angles to radians = 1/10430

	// find the error between control input and filtered IMU values:
	error[0] = (int32_t) mes->cm.control.yaw - yaw;
	error[1] = (int32_t) mes->cm.control.pitch - pitch;
	error[2] = (int32_t) mes->cm.control.roll - roll;

	// compute the derivative of the error:
	derror[0] = (int32_t) (error[0] - prev_error[0])*freq;
	derror[1] = (int32_t) (error[1] - prev_error[1])*freq;
	derror[2] = (int32_t) (error[2] - prev_error[2])*freq;

	// compute the integral of the error:
	ierror[0] = (int32_t) ((float) (error[0] + prev_error[0])/(2*freq));
	ierror[1] = (int32_t) ((float) (error[1] + prev_error[1])/(2*freq));
	ierror[2] = (int32_t) ((float) (error[2] + prev_error[2])/(2*freq));
}

void controller(pc_msg *mes){
	// set control gains:
	float Kpy = 72.4; float Kiy = 4.19; float Kdy = 313.0; // yaw
	float Kpp = 1.31; float Kip = 0.0756; float Kdp = 5.65; // pitch
	float Kpr = 1.31; float Kir = 0.0756; float Kdr = 5.65; // roll

	// define all 3 PID controllers
	yaw_command = (int32_t) ((float) Kpy*error[0]) + ((float) Kiy*ierror[0]) + ((float) Kdy*derror[0]);
	pitch_command = (int32_t) ((float) Kpp*error[1]) + ((float) Kip*ierror[1]) + ((float) Kdp*derror[1]);
	roll_command = (int32_t) ((float) Kpr*error[2]) + ((float) Kir*ierror[2]) + ((float) Kdr*derror[2]);

	// calculate motor outputs, scale them between 0 and 800:
	ae[0] = MIN(max_motor, MAX(min_motor, (int16_t) mes->cm.control.throttle + (- yaw_command + pitch_command)/10430)); 
	ae[1] = MIN(max_motor, MAX(min_motor, (int16_t) mes->cm.control.throttle + (yaw_command - roll_command)/10430)); 
	ae[2] = MIN(max_motor, MAX(min_motor, (int16_t) mes->cm.control.throttle + (- yaw_command - pitch_command)/10430)); 
	ae[3] = MIN(max_motor, MAX(min_motor, (int16_t) mes->cm.control.throttle + (yaw_command + roll_command)/10430)); 
}

int16_t* run_filters_and_control(pc_msg* msg, uint8_t mode)
{
	switch (mode) {
		case MODE_SAFE:
			ae[0] = safe_motor; ae[1] = safe_motor; ae[2] = safe_motor; ae[3] = safe_motor;
			break;
		
		case MODE_PANIC:
			ae[0] = panic_motor; ae[1] = panic_motor; ae[2] = panic_motor; ae[3] = panic_motor;
			break;

		case MODE_MANUAL:
			controller_manual(msg);
			break;

		case MODE_FULL_CONTROL:
			// filter_angles();
			// get_error(msg);
			// controller(msg);
			break;
		
		default:
			break;
	}
	update_motors();
	return ae;
}