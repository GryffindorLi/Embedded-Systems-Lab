#include "calibration.h"
#include <inttypes.h>
#include <stdbool.h>
#include "control.h"
#include <stdio.h>
#include "timers.h"
#include "config.h"

int start_calibration;
int calibration;

int32_t pitch_data[5], roll_data[5];
int16_t C_pitch_offset, C_roll_offset, C_yaw_offset;
int16_t C_pitch_slope, C_roll_slope;

int32_t counter = 0xFFFF;
int32_t calib_timer = -1;
int32_t Mean_pitch_offset, Mean_roll_offset;
int8_t calib_phase = 0;
int8_t calib_notice = 0;

// define angles
int32_t yaw, pitch, roll;

/*
 * @Author Kenrick Trip
 * @Param none.
 * @Return calibration data.
 */
void collect_data(){	
	pitch_data[calib_phase] = -pitch;
	roll_data[calib_phase] = -roll;
}

/*
 * @Author Kenrick Trip
 * @Param none.
 * @Return angle offsets.
 */
void set_offset(){
    if (calib_phase == 0) {
        C_yaw_offset = -yaw;
        C_pitch_offset = -pitch;
        C_roll_offset = -roll;
    }
}

/*
 * @Author Kenrick Trip
 * @Param none.
 * @Return instruction message.
 */
void send_instruction(){
    if (calib_notice) {
        switch (calib_phase) {
            case 0:
                printf("\nPlace level, hit '+' key when ready \n");
                break;
            case 1:
                printf("\nPlace nose up, hit '+' key when ready\n");
                break;
            case 2:
                printf("\nPlace nose down, hit '+' key when ready\n");
                break;
            case 3:
                printf("\nPlace sideways Left, hit '+' key when ready\n");
                break;
            case 4:
                printf("\nPlace sideways Right, hit '+' key when ready\n");
                break;
            default:
                break;
        }
        calib_notice = 0;
    }
}

/*
 * @Author Kenrick Trip
 * @Param none.
 * @Return maximum value at 90 degrees bank.
 */
void data_to_slope(){
    C_pitch_slope = (MAX(pitch_data[1], -pitch_data[1]) + MAX(pitch_data[2], -pitch_data[2]) + 2*C_pitch_offset)/2;
    C_roll_slope = (MAX(roll_data[3], -roll_data[3]) + MAX(roll_data[4], -roll_data[4]) + 2*C_roll_offset)/2;
}

/*
 * @Author Karan
 * @Param none.
 * @Return slopes and offsets of the angles.
 */
void run_calibration(uint8_t key){
    if (start_calibration > 0){
        if (start_calibration == 1) {
            calib_phase = 0;
            calib_notice = 1;
            start_calibration = 2; // during calibration
        }
        if (calib_phase < 5) {
            send_instruction();
            if (key == '+') {
                collect_data();
                set_offset();
                calib_phase += 1;
                calib_notice = 1;
                calib_timer = get_time_us();
            }
        } else {
            printf("\n---===CALIBRATION FINISHED===---\n");
            calib_phase = 0;
            calib_timer = -1;
            start_calibration = 0;
            data_to_slope();
            calibration = 1;
        }
    }
}