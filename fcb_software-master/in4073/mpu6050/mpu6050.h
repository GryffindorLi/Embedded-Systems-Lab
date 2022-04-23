#ifndef MPU6050_H_
#define MPU6050_H_

#include <inttypes.h>
#include <stdbool.h>

extern int16_t phi, theta, psi;
extern int16_t sp, sq, sr;
extern int16_t sax, say, saz;
extern uint8_t sensor_fifo_count;

extern bool sensor_mode;	// Sensor_mode = true = dmp, =false = raw mode.

void imu_init(bool dmp, uint16_t interrupt_frequency); // if dmp is true, the interrupt frequency is 100Hz - otherwise 32Hz-8kHz
void get_sensor_data(void);
bool check_sensor_int_flag(void);
void clear_sensor_int_flag(void);

#endif /* MPU6050_H_ */
