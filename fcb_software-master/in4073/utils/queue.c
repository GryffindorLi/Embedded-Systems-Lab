#include "queue.h"

void init_queue(Queue *queue)
{
	queue->first = 0;
	queue->last = QUEUE_SIZE - 1;
	queue->count = 0;
}

void enqueue(Queue *queue, uint8_t item)
{
	queue->last = (queue->last + 1) % QUEUE_SIZE;
	queue->items[queue->last] = item;
	queue->count += 1;
}

uint8_t dequeue(Queue *queue)
{
	if (queue->count > 0){
		uint8_t first_item = queue->items[queue->first];
		queue->first = (queue->first + 1) % QUEUE_SIZE;
		queue->count -= 1;	
		return first_item;
	}
	return -1;
}
