#ifndef TIMERS_H_
#define TIMERS_H_

#include <stdbool.h>
#include <inttypes.h>

void timers_init(void);
uint32_t get_time_us(void);
bool check_timer_flag(void);
void clear_timer_flag(void);


#endif /* TIMERS_H_ */
