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

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

int32_t C_pitch_offset[6], C_roll_offset[6];
int32_t counter = 0xFFFF;
int32_t idle_timer;
int32_t Mean_pitch_offset, Mean_roll_offset;
int8_t FLAG;




uint16_t motor[4];
int16_t ae[4];
bool wireless_mode;
bool yaw_control_mode;

// for IMU:
int16_t phi, theta, psi; // computed angles  (deg to int16), LSB = 182
int16_t sp, sq, sr; // x,y,z gyro (deg/s to int16), LSB = 16.4
int16_t sax, say, saz; // x,y,z accel (m/s^2 to int16), LSB = 

// angle definitions:
int32_t yaw, pitch, roll = 0;

// control variables:
int32_t error[3];
int32_t prev_error[3] = {0, 0, 0}; // initialize to 0
int32_t derror[3];
int32_t ierror[3];

int32_t yaw_command;
int32_t pitch_command;
int32_t roll_command;

// yaw control gains:
int16_t Kpy = 311, Kiy = 61, Kdy = 10; // yaw
// pitcha nd roll control gains * 1000:
int16_t Kpp = 1310, Kip = 75, Kdp = 5650; // pitch
int16_t Kpr = 1310, Kir = 75, Kdr = 5650; // roll

// throttle scalaing: 65535/800 ≈ 82
int16_t t_scale = 82;
// throttle scalaing: 65535/300 ≈ 220
int16_t t_scale_manual = 220;
// angle scalaing: 32767/65 ≈ 500
int16_t a_scale = 500;

// motor range
int16_t min_motor = 10; // minimum motor value where it starts turning
int16_t manual_max_motor = 400; // max motor value in manual mode
int16_t max_motor = 1000; // max motor value in full control mode
int16_t safe_motor = 0; // safe mode motor value
int16_t panic_motor = 300; // panic mode motor value

// frequency control loop - 1/looptime
int8_t freq = 10; // hz

// control input:
// throttle ranging from 0-65536
// yaw/pitch/roll ranging from -32767 to 32767

// keyboard control offsets
int16_t throttle_offset = 0;
int16_t yaw_offset = 0;
int16_t pitch_offset = 0;
int16_t roll_offset = 0;
int16_t yaw_p_offset = 0;
int16_t pitch_p_offset = 0;
int16_t roll_p_offset = 0;
//////////////////Calibration Mode only

void calibration (controls cont)
{
	//place upside down
	printf("Start the Calibration, Place upside down");
	
	if (FLAG ==0)
	{
		printf("Place upside");
		
		idle_timer = get_time_us();
		while (get_time_us() - idle_timer<5000000)
		{
		}
		C_pitch_offset[1] = pitch;
		C_roll_offset[1]= roll;
		FLAG =1;
	}
	
	if (FLAG ==1)
	{
		printf("Place upside down");
		
		idle_timer = get_time_us();
		while (get_time_us() - idle_timer<5000000)
		{
		}
		C_pitch_offset[2] = pitch-90;
		C_roll_offset[2]= roll;
		FLAG=2;

	}
	if (FLAG ==2)
	{
		printf("Place rocket up");
		
		idle_timer = get_time_us();
		while (get_time_us() - idle_timer<5000000)
		{
		}
		C_pitch_offset[3] = pitch-90;
		C_roll_offset[3]= roll;
		FLAG=3;
	}
	if (FLAG ==3)
	{
		printf("Place rocket down");
		
		idle_timer = get_time_us();
		while (get_time_us() - idle_timer<5000000)
		{
		}
		C_pitch_offset[4] = pitch-90;
		C_roll_offset[4]= roll;
		FLAG=4;
	}
	if (FLAG ==4)
	{
		printf("Place sideways Left");
		
		idle_timer = get_time_us();
		while (get_time_us() - idle_timer<5000000)
		{
		}
		C_pitch_offset[4] = pitch-90;
		C_roll_offset[4]= roll;
		FLAG=5;
	}
	if (FLAG ==5)
	{
		printf("Place sideways Right");
		
		idle_timer = get_time_us();
		while (get_time_us() - idle_timer<5000000)
		{
		}
		C_pitch_offset[5] = pitch-90;
		C_roll_offset[5]= roll;
		FLAG = 6;

	}

}

void update_motors(void)
{
	motor[0] = ae[0];
	motor[1] = ae[1];
	motor[2] = ae[2];
	motor[3] = ae[3];
}

// ------------- Manual Mode only ----------------
void controller_manual(controls cont){
	ae[0] = MIN(manual_max_motor, MAX(min_motor, cont.throttle / t_scale_manual + (- cont.yaw + cont.pitch) / a_scale)); 
	ae[1] = MIN(manual_max_motor, MAX(min_motor, cont.throttle / t_scale_manual + (cont.yaw - cont.roll) / a_scale)); 
	ae[2] = MIN(manual_max_motor, MAX(min_motor, cont.throttle / t_scale_manual + (- cont.yaw - cont.pitch) / a_scale)); 
	ae[3] = MIN(manual_max_motor, MAX(min_motor, cont.throttle / t_scale_manual + (cont.yaw + cont.roll) / a_scale)); 
}

// ------------- Control Mode only ----------------
void filter_angles(void){
	// angles are in degrees
	// combine gyro and accelerometer to remove drift:
	// constants are LSB values from mpu6050.c

	float gyro_rate = 0.98;
	float acc_rate = 0.02;

	float gyro_rate_yaw = 0.99;
	float acc_rate_yaw = 0.01;

	int16_t LSB_a = 182; // LSB angle values (int16 to deg)
	float LSB_av = 16.4; // LSB anglar velocity (int16 to deg/s)
	int16_t LSB_ac = 1670; // LSB acceleration (int16 to m/s^2)

	pitch = (int32_t) LSB_a*(gyro_rate*(phi/LSB_a + sp/(LSB_av*freq)) + acc_rate*(sax/LSB_ac));
	roll = (int32_t) LSB_a*(gyro_rate*(theta/LSB_a + sq/(LSB_av*freq)) + acc_rate*(say/LSB_ac));
	yaw = (int32_t) LSB_av*(gyro_rate_yaw*(sr/LSB_av) + acc_rate_yaw*(saz/LSB_ac)); // this is a rate
}

void get_error(controls cont){
	// find the error between control input and filtered IMU values:
	error[0] = (int32_t) cont.yaw - yaw;
	error[1] = (int32_t) cont.pitch - pitch;
	error[2] = (int32_t) cont.roll - roll;

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

void controller(controls cont){
	// IMU angles to radians = 1/10430
	int16_t LSB_rad = 10430;
	// yaw rate to radians/s = 1/940
	int16_t LSB_drad = 940;

	// throttle scalaing: 65535/800 ≈ 82
	int16_t t_scale = 82;

	// define all 3 PID controllers
	yaw_command = (int32_t) (Kpy*error[0] + Kiy*ierror[0] + Kdy*derror[0]) / LSB_drad; // note: we do not devide by 1000
	if (yaw_control_mode) {
		pitch_command = 0;
		roll_command = 0;
	} else {
		pitch_command = (int32_t) (Kpp*error[1] + Kip*ierror[1] + Kdp*derror[1])/(1000*LSB_rad);
		roll_command = (int32_t) (Kpr*error[2] + Kir*ierror[2] + Kdr*derror[2])/(1000*LSB_rad);
	}

	// calculate motor outputs, scale them between 0 and 800:
	ae[0] = MIN(max_motor, MAX(min_motor, (int16_t) cont.throttle/t_scale + (-yaw_command + pitch_command))); 
	ae[1] = MIN(max_motor, MAX(min_motor, (int16_t) cont.throttle/t_scale + (yaw_command - roll_command))); 
	ae[2] = MIN(max_motor, MAX(min_motor, (int16_t) cont.throttle/t_scale + (-yaw_command - pitch_command))); 
	ae[3] = MIN(max_motor, MAX(min_motor, (int16_t) cont.throttle/t_scale + (yaw_command + roll_command))); 
}

int16_t* run_filters_and_control(controls cont, uint8_t key, uint8_t mode)
{
	controls actuate_cont;
	switch (mode) {
		case MODE_SAFE:
			ae[0] = safe_motor; ae[1] = safe_motor; ae[2] = safe_motor; ae[3] = safe_motor;
			reset_offset();
			break;
		
		case MODE_PANIC:
			ae[0] += panic_motor; ae[1] = panic_motor; ae[2] = panic_motor; ae[3] = panic_motor;
			// actuate_cont.throttle = panic_motor * t_scale_manual; 
			// actuate_cont.yaw = 0;
			// actuate_cont.pitch = 0;
			// actuate_cont.roll = 0;
			// filter_angles();
			// get_error(cont);
			// controller(cont);
			break;

		case MODE_MANUAL:
			handle_keys(key);
			// filter_angles();
			// get_error(msg);
			actuate_cont = offset_controls(cont);
			controller_manual(actuate_cont);
			break;
		
		case MODE_YAW_CONTROL:
			yaw_control_mode = true;
			handle_keys(key);
			actuate_cont = offset_controls(cont);
			filter_angles();
			get_error(actuate_cont);
			controller(actuate_cont);
			yaw_control_mode = false;
			break;

		case MODE_FULL_CONTROL:
			handle_keys(key);
			actuate_cont = offset_controls(cont);
			filter_angles();
			get_error(actuate_cont);
			controller(actuate_cont);
			break;
		case MODE_CALIBRATION:	
			ae[0] = safe_motor; ae[1] = safe_motor; ae[2] = safe_motor; ae[3] = safe_motor;//motors off
			filter_angles();
			// idle_timer = get_time_us();
			// if (get_time_us() - idle_timer > 5000000)
			// 	{
			// 		idle_timer = get_time_us();
			// 	}
			calibration(cont);		
			int32_t temp1 =0;
			int32_t temp2 =0;
			for (int i=0;i<6;i++)
			{
				temp1 += C_pitch_offset[i];
				temp2 += C_roll_offset[i];
			}
			Mean_pitch_offset= temp1;
			Mean_roll_offset = temp2;
			break;
		default:
			break;
	}
	update_motors();
	return ae;
}


void handle_keys(uint8_t key) {
	switch (key) {
		case 'a':
			throttle_offset += 10;
			break;
		case 'z':
			throttle_offset -= 10;
			break;
		case 'q':
			yaw_offset -= 10;
			break;
		case 'w':
			yaw_offset += 10;
			break;
		case 72: //up
			pitch_offset += 10;
			break;
		case 80: //down
			pitch_offset -= 10;
			break;
		case 75: //left
			roll_offset += 10;
			break;
		case 77: //right
			roll_offset -= 10;
			break;
		case 'u':
			yaw_p_offset += 1;
			break;
		case 'j':
			yaw_p_offset -= 1;
			break;
		case 'i':
			roll_p_offset += 1;
			pitch_p_offset += 1;
			break;
		case 'k':
			roll_p_offset -= 1;
			pitch_p_offset -= 1;
			break;
		default:
			break;
	}
	throttle_offset = MIN(20000, MAX(-20000, throttle_offset));
	yaw_offset = MIN(20000, MAX(-20000, yaw_offset));
	pitch_offset = MIN(20000, MAX(-20000, pitch_offset));
	roll_offset = MIN(20000, MAX(-20000, roll_offset));
	yaw_p_offset = MIN(500, MAX(-500, yaw_p_offset));
	pitch_p_offset = MIN(500, MAX(-500, pitch_p_offset));
	roll_p_offset = MIN(500, MAX(-500, roll_p_offset));
}


uint16_t safeuint16pint16(uint16_t a, int16_t b) {
	if (b < 0) {
		if (a < (uint16_t) (-b)) return 0;
		else return a - (uint16_t) (-b);
	} else {
		if ((uint16_t) b > 65535 - a) return 65535;
		else return a + (uint16_t) b;
	}
}

int16_t safeint16pint16(int16_t a, int16_t b) {
	if (b > 0) {
		if (a > 0 && b > 32767 - a) return 32767;
		else return a + b;
	} else {
		if (a < 0 && b < -32768 - a) return -32768;
		else return a + b;
	}
}

controls offset_controls(controls cont) {
	controls offset_control;
	// throttle (uint16 + int16)
	offset_control.throttle = safeuint16pint16(cont.throttle, throttle_offset);
	// yaw (int16 + int16)
	offset_control.yaw = safeint16pint16(cont.yaw, yaw_offset);
	// pitch (int16 + int16)
	offset_control.pitch = safeint16pint16(cont.pitch, pitch_offset);
	// roll (int16 + int16)
	offset_control.roll = safeint16pint16(cont.roll, roll_offset);
	// yaw_p (int16 + int16)
	Kpy = safeint16pint16(Kpy, yaw_p_offset);
	// pitch_p (int16 + int16)
	Kpp = safeint16pint16(Kpp, pitch_p_offset);
	// roll_p (int16 + int16)
	Kpr = safeint16pint16(Kpr, roll_p_offset);
	return offset_control;
}

void reset_offset() {
	throttle_offset = 0;
	yaw_offset = 0;
	pitch_offset = 0;
	roll_offset = 0;
	yaw_p_offset = 0;
	roll_p_offset = 0;
	pitch_p_offset = 0;
}