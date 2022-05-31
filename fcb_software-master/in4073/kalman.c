#include "control.h"
#include <inttypes.h>
#include <stdbool.h>
#include "PC2D.h"
#include "config.h"
#include "keyboard.h"
#include "kalman.h"

// motor values
int16_t ae[4];

// for IMU:
int16_t phi, theta, psi; // computed angles  (deg to int16), LSB = 182
int16_t sp, sq, sr; // x,y,z gyro (deg/s to int16), LSB = 16.4
int16_t sax, say, saz; // x,y,z accel (m/s^2 to int16), LSB = 1670


// for yaw axis:
uint16_t c1 = 10; // change to actual value
uint32_t y[3] = {0, 0, 0}; 
uint32_t y_next[3] = {0, 0, 0}; 
uint32_t Py[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; 
uint32_t Py_next[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
uint32_t Yy[2] = {0, 0};
uint8_t Qy[3] = {2, 1, 1};
uint8_t Ry[2] = {1, 1};
uint32_t Ky[6] = {0, 0, 0, 0, 0, 0};


void yaw_state(){
    y_next[0] = y[0] + y[1]/freq - y[2]/freq;
    y_next[1] = y[1] + (c1*(-ae[0] + ae[1] - ae[2] + ae[3]));
    y_next[2] = y[2];
}

void yaw_cov(){
    Py_next[0] = Py[0] + (Py[3] - Py[6] + Py[1] - Py[2])/freq + (Py[4] - Py[7] - Py[5] + Py[8])/(freq*freq) + Qy[0];
    Py_next[1] = Py[1] + Py[4]/freq - Py[7]/freq;
    Py_next[2] = Py[2] + Py[5]/freq - Py[8]/freq;
    Py_next[3] = Py[3] + Py[4]/freq - Py[5]/freq;
    Py_next[4] = Py[4] + Qy[1];
    Py_next[5] = Py[5];
    Py_next[6] = Py[6] + Py[7]/freq - Py[8]/freq;
    Py_next[7] = Py[7];
    Py_next[8] = Py[8] + Qy[2];
}

void yaw_error(){
    Yy[0] = sr - y_next[0];
    Yy[1] = (acc_rate_yaw*(saz/LSB_acc))/1000 - y_next[1];
}

void yaw_gain(){
    Ky[0] = Py_next[0]*(Py_next[4] + Ry[0]) - Py_next[1]+Py_next[3];
    Ky[1] = Py_next[1]*(Py_next[0] + Ry[1]) - Py_next[0]+Py_next[1];
    Ky[2] = Py_next[3]*(Py_next[4] + Ry[0]) - Py_next[4]+Py_next[3];
    Ky[3] = Py_next[4]*(Py_next[0] + Ry[1]) - Py_next[3]+Py_next[1];
    Ky[4] = Py_next[6]*(Py_next[4] + Ry[0]) - Py_next[7]+Py_next[3];
    Ky[5] = Py_next[7]*(Py_next[0] + Ry[1]) - Py_next[6]+Py_next[1];
}

void yaw_update(){
    // state update:
    y[0] = y_next[0] + Ky[0]*Yy[0] + Ky[1]*Yy[1]; 
    y[1] = y_next[1] + Ky[2]*Yy[0] + Ky[3]*Yy[1]; 
    y[2] = y_next[2] + Ky[4]*Yy[0] + Ky[5]*Yy[1]; 

    // covariance update:
    Py[0] = Py_next[0]*(1-Ky[0]) - Py_next[3]*Ky[1];
    Py[1] = Py_next[1]*(1-Ky[0]) - Py_next[4]*Ky[1];
    Py[2] = Py_next[2]*(1-Ky[0]) - Py_next[5]*Ky[1];
    Py[3] = -Py_next[0]*Ky[2] + Py_next[3]*(1-Ky[3]);
    Py[4] = -Py_next[1]*Ky[2] + Py_next[4]*(1-Ky[3]);
    Py[5] = -Py_next[2]*Ky[2] + Py_next[5]*(1-Ky[3]);
    Py[6] = -Py_next[0]*Ky[4] - Py_next[3]*Ky[5] + Py_next[6];
    Py[7] = -Py_next[1]*Ky[4] - Py_next[4]*Ky[5] + Py_next[7];
    Py[8] = -Py_next[2]*Ky[4] - Py_next[5]*Ky[5] + Py_next[8];
}

// for pitch axis:
uint16_t c2 = 10; // change to actual value
uint32_t p[3] = {0, 0, 0}; 
uint32_t p_next[3] = {0, 0, 0}; 
uint32_t Pp[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; 
uint32_t Pp_next[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
uint32_t Yp[2] = {0, 0};
uint8_t Qp[3] = {2, 1, 1};
uint8_t Rp[2] = {1, 1};
uint32_t Kp[6] = {0, 0, 0, 0, 0, 0};


void pitch_state(){
    p_next[0] = p[0] + p[1]/freq - p[2]/freq;
    p_next[1] = p[1] + (c1*(-ae[0] + ae[2]))/freq;
    p_next[2] = p[2];
}

void pitch_cov(){
    Pp_next[0] = Pp[0] + (Pp[3] - Pp[6] + Pp[1] - Pp[2])/freq + (Pp[4] - Pp[7] - Pp[5] + Pp[8])/(freq*freq) + Qp[0];
    Pp_next[1] = Pp[1] + Pp[4]/freq - Pp[7]/freq;
    Pp_next[2] = Pp[2] + Pp[5]/freq - Pp[8]/freq;
    Pp_next[3] = Pp[3] + Pp[4]/freq - Pp[5]/freq;
    Pp_next[4] = Pp[4] + Qp[1];
    Pp_next[5] = Pp[5];
    Pp_next[6] = Pp[6] + Pp[7]/freq - Pp[8]/freq;
    Pp_next[7] = Pp[7];
    Pp_next[8] = Pp[8] + Qp[2];
}

void pitch_error(){
    Yp[0] = theta - p_next[0];
    Yp[1] = (gyro_rate*sq + acc_rate*(say/LSB_acc))/100 - p_next[1];
}

void pitch_gain(){
    Kp[0] = Pp_next[0]*(Pp_next[4] + Rp[0]) - Pp_next[1]+Pp_next[3];
    Kp[1] = Pp_next[1]*(Pp_next[0] + Rp[1]) - Pp_next[0]+Pp_next[1];
    Kp[2] = Pp_next[3]*(Pp_next[4] + Rp[0]) - Pp_next[4]+Pp_next[3];
    Kp[3] = Pp_next[4]*(Pp_next[0] + Rp[1]) - Pp_next[3]+Pp_next[1];
    Kp[4] = Pp_next[6]*(Pp_next[4] + Rp[0]) - Pp_next[7]+Pp_next[3];
    Kp[5] = Pp_next[7]*(Pp_next[0] + Rp[1]) - Pp_next[6]+Pp_next[1];
}

void pitch_update(){
    // state update:
    p[0] = p_next[0] + Kp[0]*Yp[0] + Kp[1]*Yp[1]; 
    p[1] = p_next[1] + Kp[2]*Yp[0] + Kp[3]*Yp[1]; 
    p[2] = p_next[2] + Kp[4]*Yp[0] + Kp[5]*Yp[1]; 

    // covariance update:
    Pp[0] = Pp_next[0]*(1-Kp[0]) - Pp_next[3]*Kp[1];
    Pp[1] = Pp_next[1]*(1-Kp[0]) - Pp_next[4]*Kp[1];
    Pp[2] = Pp_next[2]*(1-Kp[0]) - Pp_next[5]*Kp[1];
    Pp[3] = -Pp_next[0]*Kp[2] + Pp_next[3]*(1-Kp[3]);
    Pp[4] = -Pp_next[1]*Kp[2] + Pp_next[4]*(1-Kp[3]);
    Pp[5] = -Pp_next[2]*Kp[2] + Pp_next[5]*(1-Kp[3]);
    Pp[6] = -Pp_next[0]*Kp[4] - Pp_next[3]*Kp[5] + Pp_next[6];
    Pp[7] = -Pp_next[1]*Kp[4] - Pp_next[4]*Kp[5] + Pp_next[7];
    Pp[8] = -Pp_next[2]*Kp[4] - Pp_next[5]*Kp[5] + Pp_next[8];
}

// for roll axis:
uint16_t c3 = 10; // change to actual value
uint32_t r[3] = {0, 0, 0}; 
uint32_t r_next[3] = {0, 0, 0}; 
uint32_t Pr[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; 
uint32_t Pr_next[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
uint32_t Yr[2] = {0, 0};
uint8_t Qr[3] = {2, 1, 1};
uint8_t Rr[2] = {1, 1};
uint32_t Kr[6] = {0, 0, 0, 0, 0, 0};


void roll_state(){
    r_next[0] = r[0] + r[1]/freq - r[2]/freq;
    r_next[1] = r[1] + (c1*(-ae[1] + ae[3]))/freq;
    r_next[2] = r[2];
}

void roll_cov(){
    Pr_next[0] = Pr[0] + (Pr[3] - Pr[6] + Pr[1] - Pr[2])/freq + (Pr[4] - Pr[7] - Pr[5] + Pr[8])/(freq*freq) + Qr[0];
    Pr_next[1] = Pr[1] + Pr[4]/freq - Pr[7]/freq;
    Pr_next[2] = Pr[2] + Pr[5]/freq - Pr[8]/freq;
    Pr_next[3] = Pr[3] + Pr[4]/freq - Pr[5]/freq;
    Pr_next[4] = Pr[4] + Qr[1];
    Pr_next[5] = Pr[5];
    Pr_next[6] = Pr[6] + Pr[7]/freq - Pr[8]/freq;
    Pr_next[7] = Pr[7];
    Pr_next[8] = Pr[8] + Qr[2];
}

void roll_error(){
    Yr[0] = phi - r_next[0];
    Yr[1] = (gyro_rate*sp + acc_rate*(sax/LSB_acc))/100 - r_next[1];
}

void roll_gain(){
    Kr[0] = Pr_next[0]*(Pr_next[4] + Rr[0]) - Pr_next[1]+Pr_next[3];
    Kr[1] = Pr_next[1]*(Pr_next[0] + Rr[1]) - Pr_next[0]+Pr_next[1];
    Kr[2] = Pr_next[3]*(Pr_next[4] + Rr[0]) - Pr_next[4]+Pr_next[3];
    Kr[3] = Pr_next[4]*(Pr_next[0] + Rr[1]) - Pr_next[3]+Pr_next[1];
    Kr[4] = Pr_next[6]*(Pr_next[4] + Rr[0]) - Pr_next[7]+Pr_next[3];
    Kr[5] = Pr_next[7]*(Pr_next[0] + Rr[1]) - Pr_next[6]+Pr_next[1];
}

void roll_update(){
    // state update:
    r[0] = r_next[0] + Kr[0]*Yr[0] + Kr[1]*Yr[1]; 
    r[1] = r_next[1] + Kr[2]*Yr[0] + Kr[3]*Yr[1]; 
    r[2] = r_next[2] + Kr[4]*Yr[0] + Kr[5]*Yr[1]; 

    // covariance update:
    Pr[0] = Pr_next[0]*(1-Kr[0]) - Pr_next[3]*Kr[1];
    Pr[1] = Pr_next[1]*(1-Kr[0]) - Pr_next[4]*Kr[1];
    Pr[2] = Pr_next[2]*(1-Kr[0]) - Pr_next[5]*Kr[1];
    Pr[3] = -Pr_next[0]*Kr[2] + Pr_next[3]*(1-Kr[3]);
    Pr[4] = -Pr_next[1]*Kr[2] + Pr_next[4]*(1-Kr[3]);
    Pr[5] = -Pr_next[2]*Kr[2] + Pr_next[5]*(1-Kr[3]);
    Pr[6] = -Pr_next[0]*Kr[4] - Pr_next[3]*Kr[5] + Pr_next[6];
    Pr[7] = -Pr_next[1]*Kr[4] - Pr_next[4]*Kr[5] + Pr_next[7];
    Pr[8] = -Pr_next[2]*Kr[4] - Pr_next[5]*Kr[5] + Pr_next[8];
}

void run_kalman_filter(){
    // yaw:
    yaw_state();
    yaw_cov();
    yaw_error();
    yaw_gain();
    yaw_update();
    // pitch:
    pitch_state();
    pitch_cov();
    pitch_error();
    pitch_gain();
    pitch_update();
    // roll:
    roll_state();
    roll_cov();
    roll_error();
    roll_gain();
    roll_update();
}