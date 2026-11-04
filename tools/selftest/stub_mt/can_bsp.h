/* Host-side stub of APP/Compat/can_bsp.h for the multi-turn test.
   mt_selftest.py checks the copied constants against the real header. */
#ifndef STUB_MT_CAN_BSP_H
#define STUB_MT_CAN_BSP_H

#include <stdint.h>

typedef enum { CAN_DM_L_J1_ID = 0x01, CAN_DM_L_GRIP_ID = 0x07 } can_id_t;

#define P_MIN   (-12.5f)
#define P_MAX   (12.5f)
#define JOINT_ERR_FAULT_MIN       8U
#define JOINT_FEEDBACK_TIMEOUT_MS 100U
#define JOINT_ALL_MASK            ((uint8_t)0x7FU)

typedef struct
{
    uint32_t motorid;
    float    position;
    float    velocity;
    float    current;
    int      p_int;
    int      v_int;
    int      t_int;
    float    torque;
    uint32_t update_tick;
    uint32_t update_count;
    uint8_t  err;
} joint_motor_measure_t;

extern joint_motor_measure_t joint_L_motor[7];
uint8_t APP_Joint_Fresh_Mask(uint32_t now_tick);

#endif
