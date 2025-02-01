#ifndef __RINGBUFFER_H__
#define __RINGBUFFER_H__

#include "system.h"

/*环形缓冲区数据结构*/
typedef struct
{
    uint8_t  *buffer;
    uint16_t size;
    volatile uint16_t in;
    volatile uint16_t out;
} RingBuffer;

void RingBuffer_Init(RingBuffer *FIFO, uint8_t *buffer, uint16_t size);
uint16_t ringbuffer_getUsedSize(RingBuffer *fifo);
uint16_t ringbuffer_getRemainSize(RingBuffer *fifo);
uint8_t ringbuffer_isEmpty(RingBuffer *fifo);
void ringbuffer_in(RingBuffer *fifo, uint8_t *data, uint16_t len);
uint8_t ringbuffer_in_check(RingBuffer *fifo, uint8_t *data, uint16_t len);
uint16_t ringbuffer_out(RingBuffer *fifo, uint8_t *buf, uint16_t len);

#endif	//__RINGBUFFER_H__
