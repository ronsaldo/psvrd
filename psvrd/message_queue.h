#ifndef PSVRD_MESSAGE_QUEUE_H
#define PSVRD_MESSAGE_QUEUE_H

#include <psvrd.h>

#define PSVRD_MESSAGE_QUEUE_MAX_CAPACITY 64

typedef struct psvrd_message_queue_s
{
    uint32_t writePointer;
    uint32_t readPointer;
    psvrd_generic_message_t data[PSVRD_MESSAGE_QUEUE_MAX_CAPACITY];
} psvrd_message_queue_t;

int psvrd_mq_isEmpty(psvrd_message_queue_t *queue);
int psvrd_mq_isFull(psvrd_message_queue_t *queue);

int psvrd_mq_push(psvrd_message_queue_t *queue, psvrd_message_header_t *message);
psvrd_generic_message_t *psvrd_mq_top(psvrd_message_queue_t *queue);
int psvrd_mq_pop(psvrd_message_queue_t *queue);

#endif /*PSVRD_MESSAGE_QUEUE_H*/
