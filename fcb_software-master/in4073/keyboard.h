#ifndef KEYBOARD_H_
#define KEYBOARD_H_

#include <inttypes.h>
#include <stdbool.h>
#include "PC2D.h"

// keyboard control offsets
int16_t throttle_offset;
int16_t yaw_offset;
int16_t pitch_offset;
int16_t roll_offset;
int16_t height_offset;

// tuned PID values:
extern int16_t p_yaw, i_yaw, d_yaw;
extern int16_t p_pitch, i_pitch, d_pitch;
extern int16_t p_roll, i_roll, d_roll;
extern int16_t p_height, i_height, d_height;

// functions:
void handle_keys(uint8_t key);
void bound_offsets();
void update_controller_gains();
controls offset_controls(controls cont);
void reset_control_offset();
void reset_tuning_gains();

#endif /* KEYBOARD_H_ */