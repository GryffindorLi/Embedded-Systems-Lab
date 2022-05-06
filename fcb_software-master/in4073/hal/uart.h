#ifndef UART_H_
#define UART_H_

#include "../utils/queue.h"

extern Queue rx_queue;
extern Queue tx_queue;
extern int UART_watch_dog;

void uart_init(void);
void uart_put(uint8_t byte);

#endif /* HAL_UART_H_ */
