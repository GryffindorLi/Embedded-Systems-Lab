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
#include "PC2D.h"
#include <stdio.h>
#include "timers.h"
#include "calibration.h"
#include "config.h"
#include "keyboard.h"
#include "intmaths.h"
#include "math.h"

// placeholders:
uint16_t motor[4];
int16_t sqrt_motor[4];
int16_t ae[4];
int wireless_mode;
int yaw_control_mode;

// for IMU:
int16_t phi, theta, psi; // computed angles  (deg to int16), LSB = 182
int16_t sp, sq, sr; // x,y,z gyro (deg/s to int16), LSB = 16.4
int16_t sax, say, saz; // x,y,z accel (m/s^2 to int16), LSB = 1670

// angle definitions:
int32_t yaw, pitch, roll = 0;
int32_t yaw_buf[3] = {0, 0, 0};
int32_t pitch_buf[3] = {0, 0, 0};
int32_t roll_buf[3] = {0, 0, 0};

// calibration data
int16_t C_pitch_offset, C_roll_offset, C_yaw_offset;
int16_t C_pitch_slope, C_roll_slope;
int calibration = 0; // set to true if the calibration mode has been run

// control variables:
int32_t error[3];
int32_t prev_error[3] = {0, 0, 0}; // initialize to 0
int32_t derror[3];
int32_t ierror[3];
int32_t yaw_command;
int32_t pitch_command;
int32_t roll_command;

// tuned PID values:
int16_t p_yaw = Kpy, i_yaw = Kiy, d_yaw = Kdy;
int16_t p_pitch = Kpp, i_pitch = Kip, d_pitch = Kdp;
int16_t p_roll = Kpr, i_roll = Kir, d_roll = Kdr;

void update_motors(void){
	motor[0] = ae[0];
	motor[1] = ae[1];
	motor[2] = ae[2];
	motor[3] = ae[3];
}

void set_aes(uint16_t throttle, int16_t scaled_roll, int16_t scaled_pitch, int16_t scaled_yaw) {
	ae[0] = int16clamp(after_sqrt_scale * int16sqrt(set_throttle(throttle, t_scale) + scaled_pitch * 2 - scaled_yaw), min_motor, max_motor);
	ae[1] = int16clamp(after_sqrt_scale * int16sqrt(set_throttle(throttle, t_scale) + scaled_roll * 2 + scaled_yaw), min_motor, max_motor);
	ae[2] = int16clamp(after_sqrt_scale * int16sqrt(set_throttle(throttle, t_scale) - scaled_pitch * 2 - scaled_yaw), min_motor, max_motor);
	ae[3] = int16clamp(after_sqrt_scale * int16sqrt(set_throttle(throttle, t_scale) - scaled_roll * 2 + scaled_yaw), min_motor, max_motor);
}

/*
 * @Author Kenrick Trip
 * @Param cont, struct of control commands.
 * @Return scaled throttle value.
 */
int16_t set_throttle(uint16_t cont_throttle, uint16_t throttle_scale){
	int16_t throttle;

	if( cont_throttle < throttle_init ){
		throttle = 0;
	} else {
		throttle = (min_motor / after_sqrt_scale) * (min_motor / after_sqrt_scale) + cont_throttle / throttle_scale;
	}
	return throttle;
}

// ____Manual Mode only____:

/*
 * @Author Kenrick Trip
 * @Param cont, struct of control commands.
 * @Return updated motor commands in ae array.
 */
void controller_manual(controls cont){
	set_aes(cont.throttle, cont.roll / a_scale, cont.pitch / a_scale, cont.yaw / y_scale);
}

// ____Control Mode only____:

/*
 * @Author Kenrick Trip
 * @Param none.
 * @Return filtered yaw, pitch and roll angles.
 */
void filter_angles(void){
	// angles are in degrees
	// combine gyro and accelerometer to remove drift:

	for(int i=0; i<2; i++){
		yaw_buf[i+1] = yaw_buf[i];
		pitch_buf[i+1] = pitch_buf[i];
		roll_buf[i+1] = roll_buf[i];
	}

	yaw_buf[0] = (int32_t) (gyro_rate_yaw*sr + acc_rate_yaw*(saz/LSB_acc))/1000; // this is a rate
	pitch_buf[0] = (int32_t) (LSB_deg*(gyro_rate*(theta/LSB_deg + (sq*10)/(LSB_ddeg*freq)) + acc_rate*(say/LSB_acc)))/100;
	roll_buf[0] = (int32_t) (LSB_deg*(gyro_rate*(phi/LSB_deg + (sp*10)/(LSB_ddeg*freq)) + acc_rate*(sax/LSB_acc)))/100;
	
	yaw = (yaw_buf[0] + yaw_buf[1] + yaw_buf[2])/3;
	pitch = (pitch_buf[0] + pitch_buf[1] + pitch_buf[2])/3;
	roll = (roll_buf[0] + roll_buf[1] + roll_buf[2])/3;

	if( calibration == 1 ){
		yaw = yaw + C_yaw_offset;
		pitch = ((pitch + C_pitch_offset)*16384)/C_pitch_slope;
		roll = ((roll + C_roll_offset)*16384)/C_roll_slope; 
	}

	if (print_angles)
		printf("\nYaw: %ld, Pitch: %ld, Roll: %ld\n", yaw, pitch, roll);
}

/*
 * @Author Kenrick Trip
 * @Param cont, struct of control commands.
 * @Return error, error derivative and the error integral.
 */
void get_error(controls cont){
	// find the error between control input and filtered IMU values:
	error[0] = (int32_t) cont.yaw/2 - yaw; // scale yaw control input
	error[1] = (int32_t) cont.pitch - pitch; // scale pitch control input
	error[2] = (int32_t) cont.roll - roll; // scale roll control input

	// compute the derivative of the error:
	derror[0] = (int32_t) (error[0] - prev_error[0]) * freq;
	derror[1] = (int32_t) (error[1] - prev_error[1]) * freq;
	derror[2] = (int32_t) (error[2] - prev_error[2]) * freq;

	// compute the integral of the error:
	ierror[0] = (int32_t) ((float) (error[0] + prev_error[0]) / (2 * freq));
	ierror[1] = (int32_t) ((float) (error[1] + prev_error[1]) / (2 * freq));
	ierror[2] = (int32_t) ((float) (error[2] + prev_error[2]) / (2 * freq));

	// update previous error:
	prev_error[0] = error[0];
	prev_error[1] = error[1];
	prev_error[2] = error[2];
}

// /*
//  * @Author Kenrick Trip
//  * @Param cont, struct of control commands.
//  * @Return updated motor commands in ae array.
//  * @TODO: not yet tested!!!!
//  * @retrieved from: https://www.codeproject.com/Articles/69941/Best-Square-Root-Method-Algorithm-Function-Precisi
//  */
// int16_t sqrt_motors(int16_t motor_val)
// {
// 	int16_t sqrt = *(uint16_t*) &motor_val; 
// 	// set square root bias
// 	sqrt  += 127 << 23;
// 	// appraximate the square root
// 	sqrt >>= 1;

// 	return *(int16_t*) &sqrt;
// }   

/*
 * @Author Kenrick Trip
 * @Param cont, struct of control commands.
 * @Return updated motor commands in ae array.
 */
void controller(controls cont){
	if(set_throttle(cont.throttle, t_scale) < 100){
		ae[0] = ae[1] = ae[2] = ae[3] = 0;
	} else {
		// define all 3 PID controllers
		yaw_command = (int32_t) (p_yaw*error[0] + i_yaw*ierror[0] + d_yaw*derror[0]) / (100*LSB_drad); // note: do not devide by 1000

		if (yaw_control_mode) {
			pitch_command = 0;
			roll_command = 0;
		} else {
			yaw_command = 0; // disable yaw control for tuning
			pitch_command = (int32_t) (p_pitch*error[1] + i_pitch*ierror[1] + d_pitch*derror[1])/(10*LSB_rad); // note: devide by 1000
			roll_command = (int32_t) (p_roll*error[2] + i_roll*ierror[2] + d_roll*derror[2])/(10*LSB_rad); // note: devide by 1000
		}
		set_aes(cont.throttle, roll_command, pitch_command, yaw_command);
	}
}

/*
 * @Author Hanyuan
 * @Param cont, struct of control commands, key input, controller mode.
 * @Return motor output.
 */
int16_t* run_filters_and_control(controls cont, uint8_t key, uint8_t mode)
{
	controls actuate_cont;
	switch (mode) {
		case MODE_SAFE:
			ae[0] = safe_motor; ae[1] = safe_motor; ae[2] = safe_motor; ae[3] = safe_motor;
			reset_control_offset();
			break;
		
		case MODE_PANIC:
			ae[0] = MAX(MIN(ae[0], panic_motor), ae[0] - panic_rampdown_factor); 
			ae[1] = MAX(MIN(ae[1], panic_motor), ae[1] - panic_rampdown_factor);  
			ae[2] = MAX(MIN(ae[2], panic_motor), ae[2] - panic_rampdown_factor); 
			ae[3] = MAX(MIN(ae[3], panic_motor), ae[3] - panic_rampdown_factor); 
			break;

		case MODE_MANUAL:
			handle_keys(key);
			actuate_cont = offset_controls(cont);
			controller_manual(actuate_cont);
			break;

		case MODE_CALIBRATION:	
			ae[0] = safe_motor; ae[1] = safe_motor; ae[2] = safe_motor; ae[3] = safe_motor; // motors off
			filter_angles();
			run_calibration(key);
			break;

		case MODE_YAW_CONTROL:
			yaw_control_mode = 1;
			handle_keys(key);
			actuate_cont = offset_controls(cont);

			#ifdef tuning
				update_controller_gains();
			#endif

			filter_angles();
			get_error(actuate_cont);
			controller(actuate_cont);
			break;

		case MODE_FULL_CONTROL:
			yaw_control_mode = 0;
			handle_keys(key);
			actuate_cont = offset_controls(cont);

			#ifdef tuning
				update_controller_gains();
			#endif

			filter_angles();
			get_error(actuate_cont);
			controller(actuate_cont);
			break;

		default:
			break;
	}
	update_motors();
	return ae;
}