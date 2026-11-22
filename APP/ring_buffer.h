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
/*
 * 中文说明：单生产者 / 单消费者字节环（SPSC），专供 USB CDC 接收路径使用。
 *   - 生产者 = USB 中断（Arm_USB_RxPush），只写 head；
 *   - 消费者 = USB 任务（parse_rx），只写 tail；
 * 单核上两边各写自己的索引，配合编译器屏障即可无锁，因此中断里也能安全调用。
 *
 * 16 位索引自由回绕，访问时与 RING_BUFFER_SIZE-1 相与，所以容量必须是 2 的幂
 * 且不超过 32768（否则回绕后的差值判断会失效）。
 *
 * 限制：只能承载字节流，不能当多生产者队列用 —— 两个中断同时写 head 会丢数据；
 * 写入是「整包或全丢」，绝不留下半帧，被拒字节数记在 dropped 里。
 */

#ifndef __RING_BUFFER_H
#define __RING_BUFFER_H

#include <stdint.h>

#define RING_BUFFER_SIZE    4096U  // 4 KB：约可吸收十几个 100 Hz 周期内的突发接收量

// 编译期校验：容量必须是 2 的幂且不超过 16 位索引的一半
#if (RING_BUFFER_SIZE & (RING_BUFFER_SIZE - 1U)) != 0U || RING_BUFFER_SIZE > 32768U
#error "RING_BUFFER_SIZE must be a power of two <= 32768"
#endif

/* 字段：buffer 数据区、head 生产者索引、tail 消费者索引、dropped 拒收计数。
   三个索引/计数都是自由回绕的 16/32 位整数，不做取模，靠掩码与快照计算。 */
typedef struct
{
    uint8_t           buffer[RING_BUFFER_SIZE];  // 数据区；物理下标 = 索引 & (SIZE-1)
    volatile uint16_t head;         /* written by the producer only  */
    volatile uint16_t tail;         /* written by the consumer only  */
    volatile uint32_t dropped;      /* bytes refused because full    */
} RingBuffer_t;

/* Producer side (interrupt). Writes all or nothing, returns bytes written. */
/* 生产者侧（USB 中断）写入：整包或全丢，返回写入字节数。 */
uint16_t ring_buffer_write_bytes(RingBuffer_t *rb, const uint8_t *data, uint16_t len);

/* Consumer side (task). */
/* 可用字节数（生产者在跑，返回值只是快照）。消费者上下文。 */
uint16_t ring_buffer_available(const RingBuffer_t *rb);
/* 从 tail 读 len 字节但不消费；返回实际拷贝字节数。消费者上下文。 */
uint16_t ring_buffer_peek(const RingBuffer_t *rb, uint8_t *data, uint16_t len);
/* 前移 tail 丢弃 len 字节（按可用量截断）。消费者上下文。 */
void     ring_buffer_skip(RingBuffer_t *rb, uint16_t len);
/* tail 追上 head：丢弃全部未读字节。消费者上下文。 */
void     ring_buffer_flush(RingBuffer_t *rb);

/* Drop everything written before the producer's head reached `pos`
   (a value of head captured earlier). No-op if the reader is already past. */
void     ring_buffer_discard_until(RingBuffer_t *rb, uint16_t pos);  // 丢弃旧会话残留字节用；只前进、不越过 head

#endif /* __RING_BUFFER_H */
