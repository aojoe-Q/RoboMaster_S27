/**
  ******************************************************************************
  * @file    ring_buffer.c
  * @brief   Single producer / single consumer byte ring.
  ******************************************************************************
  */

#include "ring_buffer.h"

/* Keep the data copy before the index update. */
#define RB_COMPILER_BARRIER()   __asm volatile ("" ::: "memory")

#define RB_MASK                 (RING_BUFFER_SIZE - 1U)

uint16_t ring_buffer_write_bytes(RingBuffer_t *rb, const uint8_t *data, uint16_t len)
{
    if (rb == 0 || data == 0 || len == 0U)
    {
        return 0U;
    }

    uint16_t head = rb->head;
    uint16_t used = (uint16_t)(head - rb->tail);

    /* A partial frame is worse than none: refuse the whole transfer. */
    if (len > (uint16_t)(RING_BUFFER_SIZE - used))
    {
        rb->dropped += len;
        return 0U;
    }

    for (uint16_t i = 0; i < len; i++)
    {
        rb->buffer[(uint16_t)(head + i) & RB_MASK] = data[i];
    }

    RB_COMPILER_BARRIER();
    rb->head = (uint16_t)(head + len);
    return len;
}

uint16_t ring_buffer_available(const RingBuffer_t *rb)
{
    return rb ? (uint16_t)(rb->head - rb->tail) : 0U;
}

uint16_t ring_buffer_peek(const RingBuffer_t *rb, uint8_t *data, uint16_t len)
{
    if (rb == 0 || data == 0)
    {
        return 0U;
    }

    uint16_t tail = rb->tail;
    uint16_t avail = (uint16_t)(rb->head - tail);
    RB_COMPILER_BARRIER();

    if (len > avail)
    {
        len = avail;
    }

    for (uint16_t i = 0; i < len; i++)
    {
        data[i] = rb->buffer[(uint16_t)(tail + i) & RB_MASK];
    }

    return len;
}

void ring_buffer_skip(RingBuffer_t *rb, uint16_t len)
{
    if (rb == 0)
    {
        return;
    }

    uint16_t avail = (uint16_t)(rb->head - rb->tail);
    if (len > avail)
    {
        len = avail;
    }

    RB_COMPILER_BARRIER();
    rb->tail = (uint16_t)(rb->tail + len);
}

void ring_buffer_flush(RingBuffer_t *rb)
{
    if (rb != 0)
    {
        rb->tail = rb->head;
    }
}

void ring_buffer_discard_until(RingBuffer_t *rb, uint16_t pos)
{
    if (rb == 0)
    {
        return;
    }

    uint16_t tail = rb->tail;
    uint16_t ahead = (uint16_t)(pos - tail);

    /* Only move forward, and never past head. */
    if (ahead != 0U && ahead <= (uint16_t)(rb->head - tail))
    {
        RB_COMPILER_BARRIER();
        rb->tail = pos;
    }
}
