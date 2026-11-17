/* Host-side stub of APP/Arm_L_task.h.
 * Joint limits are generated from APP/Arm_L_task.h by run_selftest.py.
 */
#ifndef STUB_ARM_L_TASK_H
#define STUB_ARM_L_TASK_H

#include <stdint.h>

typedef float fp32;

/* Generated in build/, never maintained by hand. Run the Python test runner. */
#include "arm_joint_limits_generated.h"

typedef struct
{
    fp32 Initial_Total_position;
    fp32 position;
    fp32 send_set;
    fp32 Error;
    fp32 Now_set_position;
    fp32 Now_theta;
    fp32 Angle_Sum_Value;
    fp32 RC_Sum_Value;
    fp32 Key_Control_Value;
    fp32 Kp;
    fp32 Kd;
} DM_contorl_t;

typedef struct
{
    uint8_t      ArmMode;
    DM_contorl_t J1, J2, J3, J4, J5, J6, Grip;
    fp32         Total_Angle;
    fp32         pitch_position;
    fp32         roll_position;
} ArmData_t;

/* Position of ARM_MODE_PC_CONTROL in the real Arm_mode_t; run_selftest.py
   checks it against APP/Arm_L_task.h. */
#define ARM_MODE_PC_CONTROL 10

extern ArmData_t ArmData;
extern volatile uint8_t Arm_L_Ready;

#endif /* STUB_ARM_L_TASK_H */
