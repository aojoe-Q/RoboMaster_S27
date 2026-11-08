/* Host-side stub of APP/Compat/can_bsp.h: joint feedback only. */
#ifndef STUB_CAN_BSP_H
#define STUB_CAN_BSP_H

#include <stdint.h>

typedef struct
{
    uint32_t motorid;
    uint8_t status;
    float    position;
    float    velocity;
    float    current;
    int      p_int;
    int      v_int;
    int      t_int;
    float    torque;
    uint32_t update_tick;
    uint32_t update_count;
} joint_motor_measure_t;

extern joint_motor_measure_t joint_L_motor[7];

#endif /* STUB_CAN_BSP_H */
