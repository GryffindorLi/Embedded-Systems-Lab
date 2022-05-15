#include <inttypes.h>
#include <stdbool.h>

int32_t C_pitch_offset[6], C_roll_offset[6];
int32_t counter;
int32_t calib_timer;
int32_t Mean_pitch_offset, Mean_roll_offset;
int8_t calib_phase;
int8_t calib_notice;
extern int start_calibration;

// Angle definitions
extern int32_t yaw, pitch, roll;

void calibration();
void run_calibration();