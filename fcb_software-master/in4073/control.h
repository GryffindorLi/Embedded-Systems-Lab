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

//calibration mode
extern int start_calibration;
void calibration (void);

void filter_angles();
void handle_keys(uint8_t key);
controls offset_controls(controls cont);
void reset_offset();
void get_error(controls cont);
void controller(controls cont);
uint16_t safeuint16pint16(uint16_t a, int16_t b);
int16_t safeint16pint16(int16_t a, int16_t b);
int16_t* run_filters_and_control(controls cont, uint8_t key, uint8_t mode, int plugged);

#endif /* CONTROL_H_ */