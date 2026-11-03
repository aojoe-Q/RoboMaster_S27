/**
  ******************************************************************************
  * @file    ring_buffer.h
  * @brief   Single producer / single consumer byte ring.
  *
  * The producer (USB interrupt) only writes head, the consumer (USB task) only
  * writes tail, so no lock is needed on a single core. The free-running 16 bit
  * indices are masked on access; size must be a power of two <= 32768.
  ******************************************************************************
  */

#ifndef __RING_BUFFER_H
#define __RING_BUFFER_H

#include <stdint.h>

#define RING_BUFFER_SIZE    4096U

#if (RING_BUFFER_SIZE & (RING_BUFFER_SIZE - 1U)) != 0U || RING_BUFFER_SIZE > 32768U
#error "RING_BUFFER_SIZE must be a power of two <= 32768"
#endif

typedef struct
{
    uint8_t           buffer[RING_BUFFER_SIZE];
    volatile uint16_t head;         /* written by the producer only  */
    volatile uint16_t tail;         /* written by the consumer only  */
    volatile uint32_t dropped;      /* bytes refused because full    */
} RingBuffer_t;

/* Producer side (interrupt). Writes all or nothing, returns bytes written. */
uint16_t ring_buffer_write_bytes(RingBuffer_t *rb, const uint8_t *data, uint16_t len);

/* Consumer side (task). */
uint16_t ring_buffer_available(const RingBuffer_t *rb);
uint16_t ring_buffer_peek(const RingBuffer_t *rb, uint8_t *data, uint16_t len);
void     ring_buffer_skip(RingBuffer_t *rb, uint16_t len);
void     ring_buffer_flush(RingBuffer_t *rb);

/* Drop everything written before the producer's head reached `pos`
   (a value of head captured earlier). No-op if the reader is already past. */
void     ring_buffer_discard_until(RingBuffer_t *rb, uint16_t pos);

#endif /* __RING_BUFFER_H */
