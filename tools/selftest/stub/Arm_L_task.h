/* Host-side stub of APP/Arm_L_task.h.
 * The joint limits must equal the real header: run_selftest.py compares them.
 */
#ifndef STUB_ARM_L_TASK_H
#define STUB_ARM_L_TASK_H

#include <stdint.h>

typedef float fp32;

#define J1_Max_position 2.81695309f
#define J1_Min_position -2.96054893f
#define J2_Max_position 1.7732357f
#define J2_Min_position 0.0f
#define J3_Max_position 2.39008179f
#define J3_Min_position 0.0f
#define J4_Max_position 4.3f
#define J4_Min_position -4.3f
#define J5_Max_position 1.23497334f
#define J5_Min_position -1.29562798f
#define J6_Max_position 2.22337666f
#define J6_Min_position -2.25160637f
#define Grip_Max_position 0.0f
#define Grip_Min_position -12.0f

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
