/* Host-side stub of FreeRTOS task.h. The test drives the clock. */
#ifndef STUB_TASK_H
#define STUB_TASK_H

#include <stdint.h>

typedef uint32_t TickType_t;

#define taskENTER_CRITICAL() do { } while (0)
#define taskEXIT_CRITICAL()  do { } while (0)
#define pdMS_TO_TICKS(ms)    ((TickType_t)(ms))

TickType_t xTaskGetTickCount(void);
void       vTaskDelayUntil(TickType_t *prev, TickType_t inc);

#endif /* STUB_TASK_H */
