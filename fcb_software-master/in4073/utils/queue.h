#ifndef QUEUE_H_
#define QUEUE_H_

#include <inttypes.h>

#define QUEUE_SIZE 256

typedef struct
{
	uint8_t items[QUEUE_SIZE];
	uint16_t first;
	uint16_t last;
	uint16_t count;
} Queue;

void init_queue(Queue *queue);
void enqueue(Queue *queue, uint8_t item);
uint8_t dequeue(Queue *queue);

#endif /* QUEUE_H_ */
