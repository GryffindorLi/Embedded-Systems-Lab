/*------------------------------------------------------------------
 *  in4073.h -- defines, globals, function prototypes
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  July 2016
 *------------------------------------------------------------------
 */
#ifndef IN4073_H__
#define IN4073_H__

#include <inttypes.h>
#include <stdbool.h>

#define BATTERY_LEVEL 1050

extern bool demo_done;
extern uint16_t bat_volt;
int32_t yaw, pitch, roll;
int calibration;


#endif // IN4073_H__
