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
#include "intmaths.h"

// keyboard control offsets
int16_t p_offset = 0;
int16_t i_offset = 0;
int16_t d_offset = 0;

// tuned PID values:
int16_t p_yaw, i_yaw, d_yaw;
int16_t p_pitch, i_pitch, d_pitch;
int16_t p_roll, i_roll, d_roll;
int16_t p_height, i_height, d_height;

int16_t roll_offset = -3700;

/*
 * @Author Kenrick Trip
 * @Param cont, struct of control commands, key input, controller mode.
 * @Return motor output.
 */
void handle_keys(uint8_t key) {
	switch (key) {
        // controller offsets:
		case 's':
			throttle_offset += throttle_per_key;
			break;
		case 'x':
			throttle_offset -= throttle_per_key;
			break;
		case 'z':
			yaw_offset -= angle_per_key;
			break;
		case 'c':
			yaw_offset += angle_per_key;
			break;
		case 'h': 
			pitch_offset += angle_per_key;
			break;
		case 'n': 
			pitch_offset -= angle_per_key;
			break;
		case 'b': 
			roll_offset += angle_per_key;
			break;
		case 'm': 
			roll_offset -= angle_per_key;
			break;
		case 'o': 
			height_offset += height_per_key;
			break;
		case 'l': 
			height_offset -= height_per_key;
			break;

        // PID tuning offsets:
		case 'p':
			if (tuning_axis == 1)
				p_offset += (tune_offset*Kpy)/100;
			else if (tuning_axis == 2)
				p_offset += (tune_offset*Kpp)/100;
			else if (tuning_axis == 3)
				p_offset += (tune_offset*Kpr)/100;
			else if (tuning_axis == 4)
				p_offset += (tune_offset*Kph)/100;
			else
				printf("ERROR: invalid tuning axis should be (1,2,3,4)");
			break;

        case 'i':
			if (tuning_axis == 1)
				i_offset += (tune_offset*Kiy)/100;
			else if (tuning_axis == 2)
				i_offset += (tune_offset*Kip)/100;
			else if (tuning_axis == 3)
				i_offset += (tune_offset*Kir)/100;
			else if (tuning_axis == 4)
				i_offset += (tune_offset*Kih)/100;
			else
				printf("ERROR: invalid tuning axis should be (1,2,3,4)");
			break;

		case 'd':
			if (tuning_axis == 1)
				d_offset += (tune_offset*Kdy)/100;
			else if (tuning_axis == 2)
				d_offset += (tune_offset*Kdp)/100;
			else if (tuning_axis == 3)
				d_offset += (tune_offset*Kdr)/100;
			else if (tuning_axis == 4)
				d_offset += (tune_offset*Kdh)/100;
			else
				printf("ERROR: invalid tuning axis should be (1,2,3,4)");
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
	height_offset = MIN(100, MAX(-100, height_offset));

    // PID tuning offset
	p_offset = MIN(30000, MAX(0, p_offset));
    i_offset = MIN(30000, MAX(0, i_offset));
    d_offset = MIN(30000, MAX(0, d_offset));
}

/*
 * @Author Kenrick Trip
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
	// height (int16 + int16)
	offset_control.height = safeint16pint16(cont.height, height_offset);

	return offset_control;
}

/*
 * @Author Kenrick Trip
 * @Param none.
 * @Return updated PID gains.
 */
void update_controller_gains(){
	if (tuning_axis == 1){
		p_yaw = safeint16pint16(Kpy, p_offset);
		i_yaw = safeint16pint16(Kiy, i_offset);
		d_yaw = safeint16pint16(Kdy, d_offset);
		if (PID_prints)
			printf("\nP: %d, I: %d, D: %d\n", p_yaw, i_yaw, d_yaw);
	}
	else if (tuning_axis == 2){
		p_pitch = safeint16pint16(Kpp, p_offset);
		i_pitch = safeint16pint16(Kip, i_offset);
		d_pitch = safeint16pint16(Kdp, d_offset);
		if (PID_prints)
			printf("\nP: %d, I: %d, D: %d\n", p_pitch, i_pitch, d_pitch);
	}
	else if (tuning_axis == 3){
		p_roll = safeint16pint16(Kpr, p_offset);
		i_roll = safeint16pint16(Kir, i_offset);
		d_roll = safeint16pint16(Kdr, d_offset);
		if (PID_prints)
			printf("\nP: %d, I: %d, D: %d\n", p_roll, i_roll, d_roll);
	}
	else if (tuning_axis == 4){
		p_height = safeint16pint16(Kph, p_offset);
		i_height = safeint16pint16(Kih, i_offset);
		d_height = safeint16pint16(Kdh, d_offset);
		if (PID_prints)
			printf("\nP: %d, I: %d, D: %d\n", p_height, i_height, d_height);
	}
	else
		printf("ERROR: invalid tuning axis should be (1,2,3,4)");
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
	p_offset = 0;
    i_offset = 0;
    d_offset = 0;
}