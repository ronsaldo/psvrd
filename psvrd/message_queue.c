#include "message_queue.h"
#include <string.h>

int psvrd_mq_isEmpty(psvrd_message_queue_t *queue)
{
    return queue->readPointer == queue->writePointer;
}

int psvrd_mq_isFull(psvrd_message_queue_t *queue)
{
    return (queue->writePointer + 1) % PSVRD_MESSAGE_QUEUE_MAX_CAPACITY == queue->readPointer;
}

int psvrd_mq_push(psvrd_message_queue_t *queue, psvrd_message_header_t *message)
{
    if(psvrd_mq_isFull(queue))
        return 0;

    memcpy(&queue->data[queue->writePointer], message, message->length);
    queue->writePointer = (queue->writePointer + 1) % PSVRD_MESSAGE_QUEUE_MAX_CAPACITY;
    return 1;
}

psvrd_generic_message_t *psvrd_mq_top(psvrd_message_queue_t *queue)
{
    if(psvrd_mq_isEmpty(queue))
        return NULL;
    return &queue->data[queue->readPointer];
}

int psvrd_mq_pop(psvrd_message_queue_t *queue)
{
    if(psvrd_mq_isEmpty(queue))
        return 0;

    queue->readPointer = (queue->readPointer + 1) % PSVRD_MESSAGE_QUEUE_MAX_CAPACITY;
    return 1;
}
