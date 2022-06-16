#include <inttypes.h>
#include <stdbool.h>
#include <inttypes.h>
#include <stdio.h>

extern int16_t ae[4];
// Angle definitions
extern int32_t yaw, pitch, roll;

extern int16_t acc_offsets[3];
extern int16_t gyro_offsets[3];

// for yaw axis:
int16_t c1;
int32_t y[3]; 
int32_t y_next[3];
int32_t Py[9];
int32_t Py_next[9];
int32_t Yy[2];
int8_t Qy[3];
int8_t Ry[2];
int32_t Ky[6];

// for pitch axis:
int16_t c2;
int32_t p[3]; 
int32_t p_next[3];
int32_t Pp[9];
int32_t Pp_next[9];
int32_t Yp[2];
int16_t Qp[3];
int16_t Rp[2];
int32_t Kp[6];

// for roll axis:
int16_t c3;
int32_t r[3]; 
int32_t r_next[3];
int32_t Pr[9];
int32_t Pr_next[9];
int32_t Yr[2];
int16_t Qr[3];
int16_t Rr[2];
int32_t Kr[6];

// functions:
void set_angles();
void run_kalman_filter();

// yaw:
void yaw_state();
void yaw_cov();
void yaw_error();
void yaw_gain();
void yaw_update();
// pitch:
void pitch_state();
void pitch_cov();
void pitch_error();
void pitch_gain();
void pitch_update();
// roll:
void roll_state();
void roll_cov();
void roll_error();
void roll_gain();
void roll_update();