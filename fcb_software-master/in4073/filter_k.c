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

// For Kalman Filter
int16_t e[2], bias[2];//error terms 
int16_t e_r[2], bias_r[2];
int16_t e_y[2], bias_y[2];
int16_t sp_scaled[2],phi_scaled[2], sax_scaled[2];
int16_t sq_scaled[2],theta_scaled[2], say_scaled[2];
int16_t sr_scaled[2], psi_scaled[2], saz_scaled[2];
int16_t C1 =100;
int32_t C2= 1000000; //constants

// placeholders:
uint16_t motor[4];
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

/*
 * @Author Karan Pathak
 * @Param none. Moving Avergae with gyro bias correction using accelerometer outputs
 * @Return filtered  pitch angles.
 */
void filter_k(void)
{		
		//pitch
		sp_scaled[0] = sp/LSB_ddeg;
		phi_scaled[0] = phi/LSB_deg;
		//roll
		theta_scaled[0]= theta/LSB_deg;
		sq_scaled[0] = sp/LSB_ddeg;
		//yaw
		sr_scaled[0] = sr/LSB_ddeg;
		for (int j=0; j<2; j++)
		{
			pitch_buf[j+1] = sp_scaled[j] - bias[j];
			phi_scaled[j+1] = phi_scaled[j] + (pitch_buf[j+1]/freq);
			e[j+1] =  phi_scaled[j+1] - sax_scaled[j+1];
			phi_scaled[j+1] = phi_scaled[j+1] - e[j+1]/C1;
			bias[j+1] = bias[j] + (e[j+1]/freq)/C2;
		}
		pitch = (pitch_buf[0] + pitch_buf[1] + pitch_buf[2])/3;

		for (int j=0; j<2; j++)
		{
			roll_buf[j+1] = sq_scaled[j] - bias_r[j];
			theta_scaled[j+1] = theta_scaled[j] + (roll_buf[j+1]/freq);
			e_r[j+1] =  theta_scaled[j+1] - say_scaled[j+1];
			theta_scaled[j+1] = theta_scaled[j+1] - e_r[j+1]/C1;
			bias_r[j+1] = bias_r[j] + (e_r[j+1]/freq)/C2;
		}
		roll = (roll_buf[0] + roll_buf[1] + roll_buf[2])/3;
		for (int j=0; j<2; j++)
		{
			yaw_buf[j+1] = sr_scaled[j] - bias_y[j];
			psi_scaled[j+1] = psi_scaled[j] + (yaw_buf[j+1]/freq);
			e_y[j+1] =  psi_scaled[j+1] - saz_scaled[j+1];
			psi_scaled[j+1] = psi_scaled[j+1] - e_y[j+1]/C1;
			bias_y[j+1] = bias_y[j] + (e_y[j+1]/freq)/C2;
		}

		yaw = (yaw_buf[0] + yaw_buf[1] + yaw_buf[2])/3;

	if( calibration == 1 ){
			pitch = ((pitch + C_pitch_offset)*16384)/C_pitch_slope;
			yaw = yaw + C_yaw_offset;
			roll = ((roll + C_roll_offset)*16384)/C_roll_slope; 
	}
}
