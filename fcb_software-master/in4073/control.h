#ifndef CONTROL_H_
#define CONTROL_H_

#include <inttypes.h>
#include <stdbool.h>
#include "PC2D.h"

extern uint16_t motor[4];
extern int16_t ae[4];
extern int wireless_mode;

// for IMU:
extern int16_t phi, theta, psi;
extern int16_t sp, sq, sr;
extern int16_t sax, say, saz;

// Angle definitions
extern int32_t yaw, pitch, roll;

// calibration data
int16_t C_pitch_offset, C_roll_offset, C_yaw_offset;
int16_t C_pitch_slope, C_roll_slope;
int calibration;

// tuned PID values:
int16_t p_yaw, i_yaw, d_yaw;
int16_t p_pitch, i_pitch, d_pitch;
int16_t p_roll, i_roll, d_roll;

// functions:
void update_motors(void);
int16_t set_throttle(controls cont, uint16_t throttle_scale);
void controller_manual(controls cont);
void filter_angles();
void get_error(controls cont);
void tune_controller();
void controller(controls cont);
int16_t* run_filters_and_control(controls cont, uint8_t key, uint8_t mode);

#endif /* CONTROL_H_ */