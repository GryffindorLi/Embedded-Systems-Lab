#include "control.h"
#include <inttypes.h>
#include <stdbool.h>
#include "PC2D.h"
#include "config.h"
#include "keyboard.h"

// init_filter:
uint32_t x[6] = {0, 0, 0, 0, 0, 0}; 
uint32_t x_min[6] = {0, 0, 0, 0, 0, 0}; 
uint32_t P[36] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 
uint32_t P_min[36] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint32_t K[36] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint16_t cov_Q = 1;


void project_state(controls cont){
    x_min[0] = x[0] + x[3]/freq + cont.yaw;
    x_min[1] = x[1] + x[4]/freq + cont.pitch;
    x_min[2] = x[2] + x[5]/freq + cont.roll;
    x_min[3] = x[3];
    x_min[4] = x[4];
    x_min[5] = x[5];
}

void project_cov(){
    for(int i=0; i<3; i++){
        P_min[i*6] = P[i*6] + P[18+i*6]/freq + P[3+i*6]/freq + P[21+i*6]/(freq*freq) + cov_Q;
        P_min[1+i*6] = P[1+i*6] + P[19+i*6]/freq + P[4+i*6]/freq + P[22+i*6]/(freq*freq);
        P_min[2+i*6] = P[2+i*6] + P[20+i*6]/freq + P[5+i*6]/freq + P[23+i*6]/(freq*freq);
        P_min[3+i*6] = P[3+i*6] + P[21+i*6]/freq;
        P_min[4+i*6] = P[4+i*6] + P[22+i*6]/freq;
        P_min[5+i*6] = P[5+i*6] + P[23+i*6]/freq;
    }

    for(int i=0; i<3; i++){
        P_min[18+i*6] = P[18+i*6] + P[21+i*6]/freq + cov_Q;
        P_min[19+i*6] = P[19+i*6] + P[22+i*6]/freq;
        P_min[20+i*6] = P[20+i*6] + P[23+i*6]/freq;
        P_min[21+i*6] = P[21+i*6];
        P_min[22+i*6] = P[22+i*6];
        P_min[23+i*6] = P[23+i*6];
    }
}

void kalman_gain(){
    K[0] = 0;
}

void state_update(uint32_t yaw, uint32_t pitch, uint32_t roll, uint32_t d_yaw, uint32_t d_pitch, uint32_t d_roll){
    for(int i=0; i<6; i++){
        x[i] = x_min[i] + K[i*6]*(yaw - x_min[i]) + K[1+i*6]*(pitch - x_min[1+i]) + K[2+i*6]*(roll - x_min[2+i]) + K[3+i*6]*d_yaw + K[4+i*6]*d_pitch + K[5+i*6]*d_roll;
    }
}

void cov_update(){
    
}