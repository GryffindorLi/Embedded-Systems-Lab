#include "keyboard.h"
#include "control.h"
#include <inttypes.h>
#include <stdbool.h>
#include "in4073.h"
#include "utils/quad_ble.h"
#include "utils/queue.h"
#include "mpu6050/mpu6050.h"
#include "PC2D.h"
#include <stdio.h>
#include "config.h"

// keyboard control offsets
int16_t throttle_offset = 0;
int16_t yaw_offset = 0;
int16_t pitch_offset = 0;
int16_t roll_offset = 0;
int16_t yaw_p_offset = 0;
int16_t pitch_p_offset = 0;
int16_t roll_p_offset = 0;
int16_t yaw_i_offset = 0;
int16_t pitch_i_offset = 0;
int16_t roll_i_offset = 0;
int16_t yaw_d_offset = 0;
int16_t pitch_d_offset = 0;
int16_t roll_d_offset = 0;

// tuned PID values:
int16_t p_yaw, i_yaw, d_yaw;
int16_t p_pitch, i_pitch, d_pitch;
int16_t p_roll, i_roll, d_roll;

/*
 * @Author Hanyuan
 * @Param cont, struct of control commands, key input, controller mode.
 * @Return motor output.
 */
void handle_keys(uint8_t key) {
	switch (key) {
        // controller offsets:
		case 'a':
			throttle_offset += throttle_per_key;
			break;
		case 'z':
			throttle_offset -= throttle_per_key;
			break;
		case 'q':
			yaw_offset -= angle_per_key;
			break;
		case 'w':
			yaw_offset += angle_per_key;
			break;
		case '!': //up
			pitch_offset += angle_per_key;
			break;
		case '@': //down
			pitch_offset -= angle_per_key;
			break;
		case '#': //left
			roll_offset += angle_per_key;
			break;
		case '$': //right
			roll_offset -= angle_per_key;
			break;

        // PID tuning offsets:
		case 'u':
			yaw_p_offset += (tune_offset*Kpy)/100;
			break;
		case 'j':
			yaw_p_offset -= (tune_offset*Kpy)/100;
			break;
		case 'i':
			yaw_i_offset += (tune_offset*Kiy)/100;
			break;
		case 'k':
			yaw_i_offset -= (tune_offset*Kiy)/100;
			break;
        case 'l':
			yaw_d_offset += (tune_offset*Kdy)/100;
			break;
		case 'm':
			yaw_d_offset -= (tune_offset*Kdy)/100;
			break;

        case 'n':
			pitch_p_offset += (tune_offset*Kpp)/100;
			break;
		case 'o':
			pitch_p_offset -= (tune_offset*Kpp)/100;
			break;
        case 's':
			pitch_i_offset += (tune_offset*Kip)/100;
			break;
		case 'b':
			pitch_i_offset -= (tune_offset*Kip)/100;
			break;
        case 'c':
			pitch_d_offset += (tune_offset*Kdp)/100;
			break;
		case 'd':
			pitch_d_offset -= (tune_offset*Kdp)/100;
			break;
        
        case 'e':
			roll_p_offset += (tune_offset*Kpr)/100;
			break;
		case 'f':
			roll_p_offset -= (tune_offset*Kpr)/100;
			break;
		case 'g':
			roll_i_offset += (tune_offset*Kir)/100;
			break;
		case 'h':
			roll_i_offset -= (tune_offset*Kir)/100;
			break;
        case '+':
			roll_d_offset += (tune_offset*Kdr)/100;
			break;
		case '-':
			roll_d_offset -= (tune_offset*Kdr)/100;
			break;

        // resets
        case 'r':
			reset_control_offset();
			break;
		case 't':
			reset_tuning_gains();
			break;

		default:
			break;
	}
    bound_offsets();
}

/*
 * @Author Kenrick Trip
 * @Param none.
 * @Return bounded offsets.
 */
void bound_offsets(){
    // controller offset
	throttle_offset = MIN(20000, MAX(-20000, throttle_offset));
	yaw_offset = MIN(20000, MAX(-20000, yaw_offset));
	pitch_offset = MIN(20000, MAX(-20000, pitch_offset));
	roll_offset = MIN(20000, MAX(-20000, roll_offset));

    // PID tuning offset
	yaw_p_offset = MIN(1000, MAX(-1000, yaw_p_offset));
    yaw_i_offset = MIN(1000, MAX(-1000, yaw_p_offset));
    yaw_d_offset = MIN(1000, MAX(-1000, yaw_p_offset));

	pitch_p_offset = MIN(1000, MAX(-1000, pitch_p_offset));
    pitch_i_offset = MIN(1000, MAX(-1000, pitch_p_offset));
    pitch_d_offset = MIN(1000, MAX(-1000, pitch_p_offset));

	roll_p_offset = MIN(1000, MAX(-1000, roll_p_offset));
    roll_i_offset = MIN(1000, MAX(-1000, roll_p_offset));
    roll_d_offset = MIN(1000, MAX(-1000, roll_p_offset));
}

/*
 * @Author Hanyuan
 * @Param cont, struct of control commands, key input, controller mode.
 * @Return motor output.
 */
controls offset_controls(controls cont) {
	controls offset_control;
	// throttle (uint16 + int16)
	offset_control.throttle = safeuint16pint16(cont.throttle, throttle_offset);
	// yaw (int16 + int16)
	offset_control.yaw = safeint16pint16(cont.yaw, yaw_offset);
	// pitch (int16 + int16)
	offset_control.pitch = safeint16pint16(cont.pitch, pitch_offset);
	// roll (int16 + int16)
	offset_control.roll = safeint16pint16(cont.roll, roll_offset);

	return offset_control;
}

/*
 * @Author Kenrick Trip
 * @Param none.
 * @Return updated PID gains.
 */
void update_controller_gains(){
    // yaw_p (int16 + int16)
	p_yaw = safeint16pint16(Kpy, yaw_p_offset);
    // yaw_i (int16 + int16)
	i_yaw = safeint16pint16(Kiy, yaw_i_offset);
    // yaw_d (int16 + int16)
	d_yaw = safeint16pint16(Kdy, yaw_d_offset);

	// pitch_p (int16 + int16)
	p_pitch = safeint16pint16(Kpp, pitch_p_offset);
    // pitch_i (int16 + int16)
	i_pitch = safeint16pint16(Kip, pitch_i_offset);
    // pitch_d (int16 + int16)
	d_pitch = safeint16pint16(Kdp, pitch_d_offset);

    // roll_p (int16 + int16)
	p_roll = safeint16pint16(Kpr, roll_p_offset);
    // roll_i (int16 + int16)
	i_roll = safeint16pint16(Kir, roll_i_offset);
    // roll_d (int16 + int16)
	d_roll = safeint16pint16(Kdr, roll_d_offset);
}

/*
 * @Author Hanyuan
 * @Param none.
 * @Return all control offsets set to 0.
 */
void reset_control_offset() {
	throttle_offset = 0;
	yaw_offset = 0;
	pitch_offset = 0;
	roll_offset = 0;
}

/*
 * @Author Kenrick Trip
 * @Param none.
 * @Return all tuning set to config file.
 */
void reset_tuning_gains() {
	yaw_p_offset = 0;
    yaw_i_offset = 0;
    yaw_d_offset = 0;

    pitch_p_offset = 0;
    pitch_i_offset = 0;
    pitch_d_offset = 0;

	roll_p_offset = 0;
    roll_i_offset = 0;
    roll_d_offset = 0;
}