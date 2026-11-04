/* Host-side stub of FreeRTOS task.h for the multi-turn test. */
#ifndef STUB_MT_TASK_H
#define STUB_MT_TASK_H

#include <stdint.h>

typedef uint32_t TickType_t;
#define taskENTER_CRITICAL() do { } while (0)
#define taskEXIT_CRITICAL()  do { } while (0)
TickType_t xTaskGetTickCount(void);

#endif
