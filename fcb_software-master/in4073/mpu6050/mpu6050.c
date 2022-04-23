/*------------------------------------------------------------------
 *  mpu_wrapper.c -- invensense sdk setup
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  July 2016
 *------------------------------------------------------------------
 */
#include "mpu6050.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "ml.h"
#include "gpio.h"
#include "nrf_gpio.h"
#include "math.h"
#include "twi.h"

int16_t phi, theta, psi;
int16_t sp, sq, sr;
int16_t sax, say, saz;
uint8_t sensor_fifo_count;

bool sensor_mode;	// Sensor_mode = true = dmp, =false = raw mode.

void update_euler_from_quaternions(int32_t *quat) 
{
	// The quaternions we receive from the dmp are in the Q30 fixed point format
	float w, x, y, z;

	// Conversion to floating point
	w = (float) quat[0] / ((float)(1L << 30));
	x = (float) quat[1] / ((float)(1L << 30));
	y = (float) quat[2] / ((float)(1L << 30));
	z = (float) quat[3] / ((float)(1L << 30));

	// Conversion to euler angles (https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles)
	// The result is in radians. degrees = rad*180/PI
	// We are using int16_t, thus ~182 LSB per degree
	// 180/PI*(65535/360) = 10430,219195527

	// roll (x-axis rotation)
	double sinr_cosp = 2 * (w * x + y * z);
	double cosr_cosp = 1 - 2 * (x * x + y * y);
	phi = atan2(sinr_cosp, cosr_cosp) * 10430;

	// pitch (y-axis rotation)
	double sinp = 2 * (w * y - z * x);
	if (abs(sinp) >= 1) {
		theta = copysign(M_PI / 2, sinp) * 10430; // use 90 degrees if out of range
	} else {
		theta = asin(sinp) * 10430;
	}

	// yaw (z-axis rotation)
	double siny_cosp = 2 * (w * z + x * y);
	double  cosy_cosp = 1 - 2 * (y * y + z * z);
	psi = atan2(siny_cosp, cosy_cosp) * 10430;
}

// reading & conversion takes 3.5 ms (still lots of time till 10?)
void get_sensor_data(void)
{
	int8_t read_stat;
	int16_t gyro[3], accel[3], dmp_sensors;
	int32_t quat[4];
	uint8_t sensors;

	if(sensor_mode) {
		if (!(read_stat = dmp_read_fifo(gyro, accel, quat, NULL, &dmp_sensors, &sensor_fifo_count))) {
			if (dmp_sensors & INV_WXYZ_QUAT) {
				update_euler_from_quaternions(quat);
			}
			//16.4 LSB/deg/s (+-2000 deg/s)
			if (dmp_sensors & INV_XYZ_GYRO) {
				sp = gyro[0];
				sq = gyro[1];
				sr = gyro[2];
			}
			//	16384 LSB/g (+-2g)
			if (dmp_sensors & INV_XYZ_ACCEL) {
				sax = accel[0];
				say = accel[1];
				saz = accel[2];
			}
		} else {
			printf("Error reading dmp sensor fifo: %d\n", read_stat);
		}
	} else {
		if (!(read_stat = mpu_read_fifo(gyro, accel, NULL, &sensors, &sensor_fifo_count))) {
			//16.4 LSB/deg/s (+-2000 deg/s)
			if (sensors & INV_XYZ_GYRO) {
				sp = gyro[0];
				sq = gyro[1];
				sr = gyro[2];
			}
			//	16384 LSB/g (+-2g)
			if (sensors & INV_XYZ_ACCEL) {
				sax = accel[0];
				say = accel[1];
				saz = accel[2];
			}
		} else {
			printf("Error reading raw sensor fifo: %d\n", read_stat);
		}
	}
}


bool check_sensor_int_flag(void)
{
	if (nrf_gpio_pin_read(INT_PIN) || sensor_fifo_count)
		return true;
	return false;
}

void imu_init(bool dmp, uint16_t freq)
{
	static int8_t gyro_orientation[9] = {
			1, 0, 0,
			0, 1, 0,
			0, 0, 1
	};

	// tap feature is there to set freq to 100Hz, a bug provided by invensense :)
	uint16_t dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_RAW_GYRO | DMP_FEATURE_TAP;

	sensor_mode = dmp;

	//mpu	
	printf("\rmpu init result: %d\n", mpu_init(NULL));
	printf("\rmpu set sensors: %d\n", mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL));
	printf("\rmpu conf fifo  : %d\n", mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL));
	mpu_set_int_level(0);
	mpu_set_int_latched(1);

	if (dmp) {
		printf("\r\ndmp load firm  : %d\n", dmp_load_motion_driver_firmware());
		printf("\rdmp set orient : %d\n", dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)));

		printf("\rdmp en features: %d\n", dmp_enable_feature(dmp_features));
		printf("\rdmp set rate   : %d\n", dmp_set_fifo_rate(100));
		printf("\rdmp set state  : %d\n", mpu_set_dmp_state(1));
		printf("\rdlpf set freq  : %d\n", mpu_set_lpf(10));
	} else {
		unsigned char data = 0;
		printf("\rdisable dlpf   : %d\n", i2c_write(0x68, 0x1A, 1, &data));
		// if dlpf is disabled (0 or 7) then the sample divider that feeds the fifo is 8kHz (derrived from gyro).
		data = 8000 / freq - 1;
		printf("\rset sample rate: %d\n", i2c_write(0x68, 0x19, 1, &data));
	}
}
