#ifndef KEYBOARD_H_
#define KEYBOARD_H_

#include <inttypes.h>
#include <stdbool.h>
#include "PC2D.h"

// keyboard control offsets
int8_t throttle_offset;
int8_t yaw_offset;
int8_t pitch_offset;
int8_t roll_offset;
int16_t yaw_p_offset;
int16_t pitch_p_offset;
int16_t roll_p_offset;
int16_t yaw_i_offset;
int16_t pitch_i_offset;
int16_t roll_i_offset;
int16_t yaw_d_offset;
int16_t pitch_d_offset;
int16_t roll_d_offset;

// tuned PID values:
extern int16_t p_yaw, i_yaw, d_yaw;
extern int16_t p_pitch, i_pitch, d_pitch;
extern int16_t p_roll, i_roll, d_roll;

// functions:
void handle_keys(uint8_t key);
void bound_offsets();
void update_controller_gains();
controls offset_controls(controls cont);
void reset_control_offset();
void reset_tuning_gains();

#endif /* KEYBOARD_H_ */