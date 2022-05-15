#include <inttypes.h>
#include <stdbool.h>

extern int16_t C_pitch_offset, C_roll_offset, C_yaw_offset;
extern int16_t C_pitch_slope, C_roll_slope;
extern int start_calibration;
extern int calibration;

int32_t pitch_data[5], roll_data[5];
int32_t counter;
int32_t calib_timer;
int32_t Mean_pitch_offset, Mean_roll_offset;
int8_t calib_phase;
int8_t calib_notice;

// Angle definitions
extern int32_t yaw, pitch, roll;

void collect_data();
void set_offset();
void run_calibration();
void data_to_slope();
void send_instruction();