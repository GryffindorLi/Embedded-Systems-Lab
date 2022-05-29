#include <inttypes.h>
#include <stdbool.h>

extern int16_t ae[4];

// for yaw axis:
uint16_t c1;
uint32_t y[3]; 
uint32_t y_next[3];
uint32_t Py[9];
uint32_t Py_next[9];
uint32_t Yy[2];
uint8_t Qy[3];
uint8_t Ry[2];
uint32_t Ky[6];

// for pitch axis:
uint16_t c2;
uint32_t p[3]; 
uint32_t p_next[3];
uint32_t Pp[9];
uint32_t Pp_next[9];
uint32_t Yp[2];
uint8_t Qp[3];
uint8_t Rp[2];
uint32_t Kp[6];

// for roll axis:
uint16_t c3;
uint32_t r[3]; 
uint32_t r_next[3];
uint32_t Pr[9];
uint32_t Pr_next[9];
uint32_t Yr[2];
uint8_t Qr[3];
uint8_t Rr[2];
uint32_t Kr[6];