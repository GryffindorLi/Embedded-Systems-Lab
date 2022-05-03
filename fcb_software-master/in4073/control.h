#ifndef CONTROL_H_
#define CONTROL_H_

#include <inttypes.h>
#include <stdbool.h>
#include "communication/PC2D.h"

extern uint16_t motor[4];
extern int16_t ae[4];
extern bool wireless_mode;

// for IMU:
extern int16_t phi, theta, psi;
extern int16_t sp, sq, sr;
extern int16_t sax, say, saz;

// for filter
float dt;
const float gyro_rate;
const float acc_rate;

// Angle definitions
int16_t yaw, pitch, roll;

// define controller gains
float Kpy; // controller P gain yaw
float Kiy; // controller I gain yaw
float Kdy; // controller D gain yaw

float Kpp; // controller P gain pitch
float Kip; // controller I gain pitch
float Kdp; // controller D gain pitch

float Kpr; // controller P gain roll
float Kir; // controller I gain roll
float Kdr; // controller D gain roll

void run_filters_and_control(pc_msg* mes, uint8_t mode);
#endif /* CONTROL_H_ */