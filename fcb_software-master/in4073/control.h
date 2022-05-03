#ifndef CONTROL_H_
#define CONTROL_H_

#include <inttypes.h>
#include <stdbool.h>

extern uint16_t motor[4];
extern int16_t ae[4];
extern bool wireless_mode;

// for IMU:
extern int16_t phi, theta, psi;
extern int16_t sp, sq, sr;
extern int16_t sax, say, saz;

// for filter
int16_t dt; // loop time period in milliseconds (ms)

// Angle definitions
int16_t yaw, pitch, roll;

void run_filters_and_control();
void filter_angles();

// check functions below
void get_error(&msg);
void controller(&msg);

#endif /* CONTROL_H_ */
