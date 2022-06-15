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
// uncomment to disable battery check when no battery is connected
// #define protect_battery


// ____PC terminal settings_____:
#define TRANSMISSION_FREQ 50
#define JOYSTICK_WATCHDOG_LIFETIME 200
#define JOYSTICK

// _____control gains_____:

// yaw control gains:
#define Kpy 2000
#define Kiy 100
#define Kdy 50

// pitch control gains * 1000:
#define Kpp 7000 // 2000
#define Kip 85 // 50
#define Kdp 300 // 200

// roll control gains * 1000:
#define Kpr 7000
#define Kir 85
#define Kdr 300

// height control gains:
#define Kph 500 // 350
#define Kih 100
#define Kdh 100


// _____scaling factors_____:

// throttle ranging from 0-65536
// yaw/pitch/roll ranging from -32767 to 32767

// throttle scaling control mode: 65535/800 ≈ 82
#define t_scale 4
// throttle scaling manual mode: 65535/300 ≈ 220
// #define t_scale_manual 220

// angle scaling: 32767/65 ≈ 500
#define a_scale 20
#define y_scale 20
// throttle_offset
#define throttle_init 5

#define after_sqrt_scale 4

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
#define min_motor 200 
// max motor value in manual mode
#define manual_max_motor 800
// max motor value in full control mode, set to 1000 after testing
#define max_motor 800
// safe mode motor value
#define safe_motor 0
// panic mode motor value
#define panic_motor 200
#define panic_rampdown_factor 1

// _____frequency control loop_____:

// 1/looptime in hz
#define freq 280
#define raw_freq 100

// _____filter_settings_____:

// select kalman filter, 1 = enable, 0 = disable:
#define use_kalman 0

// gyro and accelerometer percentages
#define gyro_rate 98
#define acc_rate 2
#define gyro_rate_yaw 1000
#define acc_rate_yaw 1
// print tuned angle values, 1 = enable, 0 = disable
#define print_angles 1

// _____tuning_settings_____:

// tuning change in percentage:
#define tune_offset 3
// 1 = yaw, 2 = pitch, 3 = roll, 4 = height

#define tuning_axis 1

// enable PID prints, 1 = enable, 0 = disable
#define PID_prints 1

// _____keyboard_control_settings_____:

// set offset gains per key hit
#define throttle_per_key 200
#define angle_per_key 100
#define height_per_key 5 // height in cm 

// _____in4073_settings_____:
#define panic_to_safe_delay 1000000
#define check_loop_time 0  

#endif /* CONFIG_H_ */