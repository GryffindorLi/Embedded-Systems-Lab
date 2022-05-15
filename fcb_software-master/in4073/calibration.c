#include "calibration.h"
#include <inttypes.h>
#include <stdbool.h>
#include "control.h"
#include <stdio.h>
#include "timers.h"

int32_t C_pitch_offset[6], C_roll_offset[6];
int32_t counter = 0xFFFF;
int32_t calib_timer = -1;
int32_t Mean_pitch_offset, Mean_roll_offset;
int8_t calib_phase = 0;
int8_t calib_notice = 0;
int start_calibration;

// define angles
int32_t yaw, pitch, roll;

// determine angle offsets

void calibration (void){	
	C_pitch_offset[calib_phase] = pitch;
	C_roll_offset[calib_phase]= roll;
	if (calib_notice) {
		switch (calib_phase) {
			case 0:
				printf("\nPlace upside\n");
				break;
			case 1:
				printf("\nPlace upside down\n");
				break;
			case 2:
				printf("\nPlace rocket up\n");
				break;
			case 3:
				printf("\nPlace rocket down\n");
				break;
			case 4:
				printf("\nPlace sideways Left\n");
				break;
			case 5:
				printf("\nPlace sideways Right\n");
				break;
			default:
				break;
		}
		calib_notice = 0;
	}
}

void run_calibration(){
    if (start_calibration > 0){
        if (start_calibration == 1) {
            calib_phase = 0;
            calib_notice = 1;
            calib_timer = get_time_us();
            start_calibration = 2; // during calibration
        }
        if (calib_timer != -1 && get_time_us() - calib_timer > 5000000) {
            calib_phase += 1;
            calib_notice = 1;
            calib_timer = get_time_us();
        }
        if (calib_phase < 6) {
            calibration();
        } else {
            printf("\n---===CALIBRATION FINISHED===---\n");
            calib_phase = 0;
            calib_timer = -1;
            start_calibration = 0;
        }

        int32_t temp1 = 0;
        int32_t temp2 = 0;
        for (int i = 0; i < 6; i++)
        {
            temp1 += C_pitch_offset[i];
            temp2 += C_roll_offset[i];
        }
    }
}