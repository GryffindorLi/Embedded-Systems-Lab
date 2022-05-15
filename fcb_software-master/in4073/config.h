#ifndef CONFIG_H_
#define CONFIG_H_

/*
 * @Author Kenrick Trip
 * file that contains all variables that can be changed.
 * created: 15-04-2022
 */

// _____enable/disable features_____:

// uncomment to diable keyboard control
#define keyboard
// uncomment to disbale serial printf statements
#define serialprintf
// uncomment to disbale tuning
#define tuning

// _____control gains_____:

// yaw control gains:
#define Kpy 311
#define Kiy 61
#define Kdy 10

// pitch control gains * 1000:
#define Kpp 1310
#define Kip 75
#define Kdp 5650

// roll control gains * 1000:
#define Kpr 1310
#define Kir 75
#define Kdr 5650

// _____scaling factors_____:

// throttle ranging from 0-65536
// yaw/pitch/roll ranging from -32767 to 32767

// throttle scaling control mode: 65535/800 ≈ 82
#define t_scale 82
// throttle scaling manual mode: 65535/300 ≈ 220
#define t_scale_manual 220
// angle scaling: 32767/65 ≈ 500
#define a_scale 500

// constants are IMU values from mpu6050.c:

// IMU angles to radians = 1/10430
#define LSB_rad 10430
// IMU rate to radians/s = 1/940
#define LSB_drad 940
// IMU angles to degrees = 1/182
#define LSB_deg 182
// IMU rate to degrees/s = 1/16.4 - we multiply by 10!
#define LSB_ddeg 164
// IMU acceleration (int16 to m/s^2) = 1/1670
#define LSB_acc 1670

// _____motor range limits_____:

// minimum motor value where it starts turning
#define min_motor 150 
// max motor value in manual mode
#define manual_max_motor 400
// max motor value in full control mode, set to 1000 after testing
#define max_motor 500
// safe mode motor value
#define safe_motor 0
// panic mode motor value
#define panic_motor 200

// _____frequency control loop_____:

// 1/looptime in hz
#define freq 50

// _____filter_settings_____:

// gyro and accelerometer percentages
#define gyro_rate 98
#define acc_rate 2
#define gyro_rate_yaw 99
#define acc_rate_yaw 1

// _____tuning_settings_____:

// tuning change in percentage:
#define tune_offset 1

// _____keyboard_control_settings_____:

// set offset gains per key hit
#define throttle_per_key 200
#define angle_per_key 100

// _____useful_functions_____:

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

#endif /* CONFIG_H_ */