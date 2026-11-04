/* Host-side stub of APP/referee_usart_task.h: master raw snapshot only. */
#ifndef STUB_MT_REFEREE_USART_TASK_H
#define STUB_MT_REFEREE_USART_TASK_H

#include <stdint.h>

typedef struct
{
  uint8_t  data[30];
  uint32_t rx_tick;
  uint32_t rx_count;
  uint8_t  online;
} master_raw_t;
extern master_raw_t Master_Raw;

#endif
