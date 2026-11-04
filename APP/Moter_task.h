#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include <stdint.h>





/* Final output gate state (0 = output allowed). Bits: 0x01 command missing
   or older than ARM_CMD_TIMEOUT_MS, 0x02 joint feedback stale / fault,
   0x04 logical coordinates not ready, 0x08 command out of range,
   0x10 latched after a powered command was blocked. */
extern uint8_t  Moter_Gate_State;
extern uint8_t  Moter_Gate_Latched;
extern uint32_t Moter_Gate_Blocked;

void Moter_Task(void const * argument);

#endif
