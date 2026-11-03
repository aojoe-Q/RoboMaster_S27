#ifndef _CAN_BSP_H
#define _CAN_BSP_H

/*
 * Compatibility layer between the S27_ARM_H7 APP/Modules sources and the
 * S27_TEST_1 COD template BSP.
 *
 * CAN mapping (kept identical to the APP original):
 *   FDCAN1 : left arm DM motors 0x01..0x07 (classic CAN, 1 Mbit)
 *   FDCAN2 : dual-board / custom controller data (0x212, 0x213, ...)
 *   FDCAN3 : arm lifting DJI motors (0x201..0x204)
 */

#include <stdint.h>
#include "stm32h7xx.h"
#include "struct_typedef.h"

/* 1 = APP arm owns FDCAN1, the template motor demo is disabled. */
#ifndef APP_ARM_ENABLE
#define APP_ARM_ENABLE 1
#endif

/* 1 = DT7/SBUS has priority; dual-board CAN is only the backup. */
#ifndef APP_RC_FALLBACK_ENABLE
#define APP_RC_FALLBACK_ENABLE 1
#endif

/* The original code used hcan_t as an alias for the HAL handle. */
typedef FDCAN_HandleTypeDef hcan_t;

/* ------------------------------ motor IDs ------------------------------ */
typedef enum
{
    /* left arm joints, FDCAN1 */
    CAN_DM_L_J1_ID   = 0x01,
    CAN_DM_L_J2_ID   = 0x02,
    CAN_DM_L_J3_ID   = 0x03,
    CAN_DM_L_J4_ID   = 0x04,
    CAN_DM_L_J5_ID   = 0x05,
    CAN_DM_L_J6_ID   = 0x06,
    CAN_DM_L_GRIP_ID = 0x07,

    /* right arm joints, kept for source compatibility (not used) */
    CAN_DM_R_J1_ID   = 0x01,
    CAN_DM_R_J2_ID   = 0x02,
    CAN_DM_R_J3_ID   = 0x03,
    CAN_DM_R_J4_ID   = 0x04,
    CAN_DM_R_J5_ID   = 0x05,
    CAN_DM_R_J6_ID   = 0x06,
    CAN_DM_R_GRIP_ID = 0x07,

    /* dual board communication, FDCAN2 */
    CAN_Board_ID_1        = 0x212,
    CAN_Board_ID_2        = 0x213,
    CAN_Board__float_ID_1 = 0x208,
    CAN_Board__float_ID_2 = 0x209,
    CAN_Board__float_ID_3 = 0x210,
    CAN_Board__float_ID_4 = 0x207,

    /* arm lifting / image transmission lifting, FDCAN3 */
    CAN_3508_L_Arm_lifting = 0x201,
    CAN_3508_R_Arm_lifting = 0x202,
    CAN_2006_L_TI_lifting  = 0x203,
    CAN_2006_R_TI_lifting  = 0x204,

    CAN_SHOOT_ALL_ID    = 0x200,
    CAN_CHASSIS_LK_ALL_ID = 0x280,
    CAN_MEASURE_ID      = 0x206,
} can_id_t;

/* --------------------------- DM motor ranges --------------------------- */
#ifndef P_MIN
#define P_MIN   (-12.5f)
#endif
#ifndef P_MAX
#define P_MAX   (12.5f)
#endif
#ifndef V_MIN
#define V_MIN   (-45.0f)
#endif
#ifndef V_MAX
#define V_MAX   (45.0f)
#endif
#ifndef KP_MIN
#define KP_MIN  (0.0f)
#endif
#ifndef KP_MAX
#define KP_MAX  (500.0f)
#endif
#ifndef KD_MIN
#define KD_MIN  (0.0f)
#endif
#ifndef KD_MAX
#define KD_MAX  (5.0f)
#endif
#ifndef T_MIN
#define T_MIN   (-54.0f)
#endif
#ifndef T_MAX
#define T_MAX   (54.0f)
#endif

#define LIMIT_MIN_MAX(x, lo, hi) \
    do {                         \
        if ((x) <= (lo)) { (x) = (lo); } \
        else if ((x) >= (hi)) { (x) = (hi); } \
    } while (0)

/* DM / HT motor command bytes used by HT_motor.c. */
#define CMD_MOTOR_MODE      0x05
#define CMD_RESET_MODE      0x06
#define CMD_ZERO_POSITION   0x07

/* Remote switch / key macros live in DT7_DR16.h (single source of truth). */

/* ------------------------------- types -------------------------------- */
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
    uint32_t update_tick;       /* RTOS tick of the last decoded feedback */
    uint32_t update_count;      /* 0 until the first feedback arrives     */
} joint_motor_measure_t;

typedef struct
{
    struct
    {
        int16_t ch[5];
        char    s[2];
    } rc;
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;
    struct
    {
        uint16_t v;
    } key;
} Rcdata_t;

typedef struct
{
    float   DM_CC_1_receive;
    float   DM_CC_2_receive;
    float   DM_CC_3_receive;
    float   DM_CC_4_receive;
    int32_t DJ_CC_1_receive;
    int32_t DJ_CC_2_receive;
    uint8_t CC_Key_grip;
    uint8_t CC_Key_J4;
} A_board_CC_data_t;

/* ------------------------------- externs ------------------------------ */
extern joint_motor_measure_t joint_L_motor[7];
extern Rcdata_t              RCData;
extern A_board_CC_data_t     CC_data;

extern void canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id,
                           uint8_t *data, uint32_t len);
extern void APP_CAN_RxHandler(FDCAN_HandleTypeDef *hcan, uint32_t id,
                              uint8_t *data);
extern void APP_RC_Fallback(void);
extern void APP_RC_Task(void const *argument);

/* Provided by DM_motor.c, shared by the old motor modules. */
extern float uint_to_float(int x_int, float x_min, float x_max, int bits);
extern int   float_to_uint(float x_float, float x_min, float x_max, int bits);

#endif /* _CAN_BSP_H */
