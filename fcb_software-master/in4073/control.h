#ifndef CONTROL_H_
#define CONTROL_H_

#include <inttypes.h>
#include <stdbool.h>

extern uint16_t motor[4];
extern int16_t ae[4];
extern bool wireless_mode;

void run_filters_and_control();

#endif /* CONTROL_H_ */
