/**
  ******************************************************************************
  * @file    ring_buffer.c
  * @brief   Single producer / single consumer byte ring.
  ******************************************************************************
  */
/*
 * 实现要点（SPSC：单生产者单消费者）：
 *   - 生产者（USB 中断）只写 head，消费者（USB 任务）只写 tail，单核上不需要
 *     关中断；靠编译器屏障 RB_COMPILER_BARRIER() 保证「先写数据、后更新索引」
 *     的顺序不被编译器/CPU 重排；
 *   - head/tail 是 16 位自由回绕计数，(head - tail) 的无符号减法在两次回绕之间
 *     始终等于占用量，索引与 RB_MASK 相与得到 0..4095 的物理下标；
 *   - 写入是「整包或全丢」（空间不足返回 0），环里永远不会出现半帧；
 *   - dropped 是累计拒收字节数，由消费者换算成增量后上报故障。
 * 本文件不做任何加锁，调用方必须遵守单生产者/单消费者约定。
 */

#include "ring_buffer.h"

/* Keep the data copy before the index update. */
#define RB_COMPILER_BARRIER()   __asm volatile ("" ::: "memory")  // 编译器屏障：保证数据拷贝先于索引更新（单核，无需原子指令）

#define RB_MASK                 (RING_BUFFER_SIZE - 1U)  // 掩码 = 4095：把自由回绕的 16 位索引折算成 0..4095 的物理下标

/* 生产者（USB 中断）写入整包字节。
   参数：rb 环；data/len 一次数据块。返回写入字节数（成功 = len）。
   空间不足时整包丢弃并累加 dropped，返回 0 —— 半帧比丢整帧更难恢复。
   上下文：单生产者，中断里调用，不加锁。 */
uint16_t ring_buffer_write_bytes(RingBuffer_t *rb, const uint8_t *data, uint16_t len)
{
    if (rb == 0 || data == 0 || len == 0U)
    {
        return 0U;
    }

    uint16_t head = rb->head;  // 生产者独占 head，读本地副本即可
    uint16_t used = (uint16_t)(head - rb->tail);  // 16 位无符号相减，回绕后仍等于占用量

    /* A partial frame is worse than none: refuse the whole transfer. */
    /* 空间不足：整包拒绝，绝不写半帧。 */
    if (len > (uint16_t)(RING_BUFFER_SIZE - used))
    {
        rb->dropped += len;  // 只累计统计，由消费者换算成增量上报
        return 0U;
    }

    for (uint16_t i = 0; i < len; i++)
    {
        rb->buffer[(uint16_t)(head + i) & RB_MASK] = data[i];  // 逐字节按掩码回绕写入
    }

    RB_COMPILER_BARRIER();  // 数据先落地，再发布新 head
    rb->head = (uint16_t)(head + len);  // 一次推进 head；消费者此时才看得到这批数据
    return len;
}

/* 消费者侧：可用字节数。只读一次 head，返回的是快照（生产者仍在跑）。 */
uint16_t ring_buffer_available(const RingBuffer_t *rb)
{
    return rb ? (uint16_t)(rb->head - rb->tail) : 0U;  // head 与 tail 都在 16 位回绕域内相减，结果正确
}

/* 从 tail 拷贝最多 len 字节但不消费。
   参数：rb 环；data 输出；len 期望长度。返回实际拷贝字节数。
   上下文：消费者（任务）调用；解析器先校验 CRC，确认整帧合法后才 skip。 */
uint16_t ring_buffer_peek(const RingBuffer_t *rb, uint8_t *data, uint16_t len)
{
    if (rb == 0 || data == 0)
    {
        return 0U;
    }

    uint16_t tail = rb->tail;
    uint16_t avail = (uint16_t)(rb->head - tail);  // 占用量的快照
    RB_COMPILER_BARRIER();  // 先读 head 再读数据，保证看到索引时数据已就绪

    if (len > avail)
    {
        len = avail;
    }

    for (uint16_t i = 0; i < len; i++)
    {
        data[i] = rb->buffer[(uint16_t)(tail + i) & RB_MASK];  // 按掩码回绕拷贝，数据区本身不移动
    }

    return len;
}

/* 前移 tail 丢弃 len 字节；超过可用量时按可用量截断。消费者上下文。 */
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
    rb->tail = (uint16_t)(rb->tail + len);  // 只由消费者推进，无需额外同步
}

/* tail 追上 head：丢弃全部未读字节，用于会话切换/初始化。消费者上下文。 */
void ring_buffer_flush(RingBuffer_t *rb)
{
    if (rb != 0)
    {
        rb->tail = rb->head;
    }
}

/* 把 tail 推进到 pos（先前抓取的 head 快照），丢弃更早写入的字节。
   只前进、不后退，且不越过 head；读指针已越过 pos 时是无操作。
   用途：新 USB 会话开始时丢掉上一会话的残留，同时保住新会话已到的数据。
   上下文：消费者（任务）调用。 */
void ring_buffer_discard_until(RingBuffer_t *rb, uint16_t pos)
{
    if (rb == 0)
    {
        return;
    }

    uint16_t tail = rb->tail;  // tail 的本地副本，避免与生产者的 head 交错读取
    uint16_t ahead = (uint16_t)(pos - tail);

    /* Only move forward, and never past head. */
    /* 只在 pos 确实在前方、且不超过 head 时才移动 tail。 */
    if (ahead != 0U && ahead <= (uint16_t)(rb->head - tail))
    {
        RB_COMPILER_BARRIER();
        rb->tail = pos;
    }
}
