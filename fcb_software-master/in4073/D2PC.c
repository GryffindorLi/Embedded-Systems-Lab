#include <stdio.h>
#include "D2PC.h"
#include <stdlib.h>
#include <string.h>

/*
 * @Author Zirui Li
 * @Param message The message with all fields being filled except the checksum field.
 * @Return the checksum of the message.
 */
int16_t cal_checksum(D2PC_message message){
    int16_t sum = 0;

    while (message.mode != 0){
        sum += (message.mode) & 1;
        message.mode >>= 1;
    }

    while (message.battery != 0){
        sum += (message.battery) & 1;
        message.battery >>= 1;
    }

    while (message.y != 0){
        sum += (message.y) & 1;
        message.y >>= 1;
    }

    while (message.p != 0){
        sum += (message.p) & 1;
        message.p >>= 1;
    }

    while (message.r != 0){
        sum += (message.r) & 1;
        message.r >>= 1;
    }

    while (message.motor1 != 0){
        sum += (message.motor1) & 1;
        message.motor1 >>= 1;
    }

    while (message.motor2 != 0){
        sum += (message.motor2) & 1;
        message.motor2 >>= 1;
    }

    while (message.motor3 != 0){
        sum += (message.motor3) & 1;
        message.motor3 >>= 1;
    }

    while (message.motor4 != 0){
        sum += (message.motor4) & 1;
        message.motor4 >>= 1;
    }

    while (message.filtered_y != 0){
        sum += (message.filtered_y) & 1;
        message.filtered_y >>= 1;
    }

    while (message.filtered_p != 0){
        sum += (message.filtered_p) & 1;
        message.filtered_p >>= 1;
    }

    while (message.filtered_r != 0){
        sum += (message.filtered_r) & 1;
        message.filtered_r >>= 1;
    }

    while (message.idx != 0){
        sum += (message.idx) & 1;
        message.idx >>= 1;
    }

    return sum;    
}