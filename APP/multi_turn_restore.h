/**
  ******************************************************************************
  * @file    multi_turn_restore.h
  * @brief   Logical joint coordinates of the seven DM motors (J1..J6, Grip).
  *
  *   p  raw motor position of this power-up       joint_L_motor[i].position
  *   b  boot offset, logical - raw               Joint_Boot_Offset[i]
  *   P  logical position, P = p + b               ArmData.Jx.position
  *   p_cmd = P_target - b                         applied in Moter_Task only
  *
  * All modes, limits, *_zero_position and the PC (USB) targets use P.
  *
  * MT_RESTORE_ENABLE = 0 (default): b stays 0, behaviour is the same as
  * before; the log still records P for later checks. Set it to 1 only after
  * each axis period MT_JOINT_PERIOD has been confirmed by power-cycle tests.
  ******************************************************************************
  */

#ifndef __MULTI_TURN_RESTORE_H
#define __MULTI_TURN_RESTORE_H

#include <stdint.h>

#ifndef MT_RESTORE_ENABLE
#define MT_RESTORE_ENABLE           0
#endif

/* Restart-equivalent period of each axis in the p coordinate, rad.
   CONFIRMED 2026-11-05, from the DM manuals themselves: every axis runs a
   -2EC motor (DM-J4310-2EC, DM-J4310P-2EC, DM-J4340P-2EC, DM-J6248P-2EC).
   All of them are dual-encoder with a SINGLE-TURN ABSOLUTE SENSOR ON THE
   OUTPUT SHAFT (manual: "output shaft single-turn absolute position, not
   afraid of losing it on power loss"), so the repeat period seen in p is
   exactly one output revolution = 2*pi. The gear ratios 10:1 / 40:1 / 48:1
   only scale the motor-side resolution, never the output-side period. */
#define MT_JOINT_PERIOD             6.28318531f
#define MT_RESTORE_RESIDUAL_MAX     0.5f    /* |P_now - P_saved| accepted, < T/2 with margin */
#define MT_RESTORE_RESIDUAL_GRIP    3.0f    /* gripper: longer travel, still < T/2 */
#define MT_RESTORE_LIMIT_MARGIN     0.1f    /* logical limit tolerance for the restored P    */
#define MT_RESTORE_ALGO_VERSION     1U

/* Save trigger: only in ARM_POWERLESS, at rest, after a real move. */
#define MT_SAVE_STILL_VEL           0.05f   /* rad/s, every axis                 */
#define MT_SAVE_STILL_MS            500U
#define MT_SAVE_MIN_DELTA           0.01f   /* rad, largest axis change          */
#define MT_SAVE_MIN_INTERVAL_MS     1000U

typedef enum
{
    MT_RESTORE_BOOT = 0,        /* before MT_Restore_Boot(): no output        */
    MT_RESTORE_DISABLED,        /* MT_RESTORE_ENABLE = 0, b = 0, output ok     */
    MT_RESTORE_READY,           /* b from a matching record, output ok         */
    MT_RESTORE_NEED_REFERENCE,  /* no / mismatching / implausible record       */
    MT_RESTORE_FAULT            /* a joint lost feedback after b was applied   */
} MT_Restore_State_t;

extern float Joint_Boot_Offset[7];
extern volatile MT_Restore_State_t MT_Restore_State;

/* Logical position of joint i (0..6). Needs can_bsp.h. */
#define JOINT_LOGICAL_POSITION(i)   (joint_L_motor[(i)].position + Joint_Boot_Offset[(i)])

/* After the motors are enabled and feeding back: set b (0 when disabled). */
void MT_Restore_Boot(void);

/* Once per Arm_L_Task period. save_window = 1 when the arm is ready, in
   ARM_POWERLESS and no gripper torque is requested. */
void MT_Restore_Process(uint8_t save_window);

/* Moter_Task: 1 while the logical coordinates may drive the motors. */
uint8_t MT_Restore_Output_Allowed(void);

/* Hash of everything a record depends on (axis order, CAN IDs, ranges,
   periods, zero points, limits, algorithm). */
uint32_t MT_Config_Id(void);

/* b for every axis from a saved P and the current raw p. Returns 1 only if
   every axis passes the residual and limit checks. Pure; exposed for tests. */
uint8_t MT_Restore_Compute(const float saved[7], const float raw[7], float b_out[7]);

#endif /* __MULTI_TURN_RESTORE_H */
