#ifndef CONTROL_H_
#define CONTROL_H_

#include <inttypes.h>
#include <stdbool.h>
#include "PC2D.h"

extern uint16_t motor[4];
extern int16_t ae[4];
extern bool wireless_mode;

// for IMU:
extern int16_t phi, theta, psi;
extern int16_t sp, sq, sr;
extern int16_t sax, say, saz;

// Angle definitions
extern int32_t yaw, pitch, roll;


void filter_angles();
void handle_keys(controls cont, uint8_t key);
void get_error(controls cont);
void controller(controls cont);
int16_t* run_filters_and_control(controls cont, uint8_t mode);

#endif /* CONTROL_H_ */