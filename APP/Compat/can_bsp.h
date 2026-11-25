/* can_bsp.h：A_core 的 CAN 兼容层头文件。
 * 提供：总线/电机 ID 枚举、达妙电机量程宏、关节反馈结构、遥控数据结构，
 *       以及 canx_send_data / APP_CAN_RxHandler / APP_RC_* 的声明。
 * 映射：FDCAN1 左臂 0x01..0x07；FDCAN2 双板 0x212/0x213/0x208..0x210/0x207；
 *       FDCAN3 抬升 DJI 0x201..0x204。
 * 注意：P/V/T_MIN..MAX 同时被 MIT 打包（DM_motor.c）和反馈解码（app_can.c）
 *       使用，改一处等于改整条控制链路的量程。 */
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
/* can_id_t：所有标准帧 ID 集中定义。
 * 达妙电机在 MIT 模式下直接用 0x01..0x07（MIT_MODE 为 0x000），
 * 其他模式在 ID 上加 POS_MODE/SPEED_MODE 偏置；右臂 ID 与左臂数值重复，
 * 仅为源码兼容保留，当前工程未使用。 */
typedef enum
{
    /* left arm joints, FDCAN1 */
    CAN_DM_L_J1_ID   = 0x01,
    CAN_DM_L_J2_ID   = 0x02,
    CAN_DM_L_J3_ID   = 0x03,
    CAN_DM_L_J4_ID   = 0x04,
    CAN_DM_L_J5_ID   = 0x05,
    CAN_DM_L_J6_ID   = 0x06,
    CAN_DM_L_GRIP_ID = 0x07, // 0x07 为夹爪

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
    CAN_Board__float_ID_4 = 0x207, // 0x207 用于 CC_Key_grip/J4

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
#define P_MIN   (-12.5f) // rad
#endif
#ifndef P_MAX
#define P_MAX   (12.5f)
#endif
#ifndef V_MIN
#define V_MIN   (-45.0f) // rad/s
#endif
#ifndef V_MAX
#define V_MAX   (45.0f)
#endif
#ifndef KP_MIN
#define KP_MIN  (0.0f)
#endif
#ifndef KP_MAX
#define KP_MAX  (700.0f) // 0..700
#endif
#ifndef KD_MIN
#define KD_MIN  (0.0f)
#endif
#ifndef KD_MAX
#define KD_MAX  (10.0f) // 0..10
#endif
#ifndef T_MIN
#define T_MIN   (-54.0f) // N·m
#endif
#ifndef T_MAX
#define T_MAX   (54.0f)
#endif

/* LIMIT_MIN_MAX：就地夹取宏（x 会被改写）。
 * 先判下界再判上界；x/lo/hi 都会被多次求值，勿传带副作用的表达式。 */
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
/* joint_motor_measure_t：单个关节的反馈快照。
 * p_int/v_int/t_int 为协议原始整数，position/velocity/torque 为换算后的
 * 物理量；update_tick/update_count 用于上层判断反馈是否超时。 */
typedef struct
{
    uint32_t motorid;
    uint8_t status; /* DM feedback high nibble: 1 = enabled */
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

/* Rcdata_t：APP 层使用的遥控数据视图。
 * rc.ch[0..4] 为 5 个通道；rc.s[0] 右拨杆、rc.s[1] 左拨杆
 * （与 remote_ctrl 的约定一致）；mouse/key 为双板或 DT7 的鼠标键盘量。 */
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

/* A_board_CC_data_t：双板经 FDCAN2 发来的自定义控制器数据。
 * DM_CC_1..4 为 4 路 float，DJ_CC_1/2 为 int32（按内存原样拷贝）；
 * CC_Key_grip/CC_Key_J4 为两个按键字节。 */
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
/* 全局对象与函数声明：joint_L_motor 定义在 Arm_L_task.c，
 * RCData/CC_data 与几个函数定义在 app_can.c。 */
extern joint_motor_measure_t joint_L_motor[7];
extern Rcdata_t              RCData;
extern A_board_CC_data_t     CC_data;

extern HAL_StatusTypeDef canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id,
                           uint8_t *data, uint32_t len);
extern void APP_CAN_RxHandler(FDCAN_HandleTypeDef *hcan, uint32_t id,
                              uint8_t *data, uint32_t tick_ms);
extern void APP_RC_Fallback(void);
extern void APP_RC_Task(void const *argument);

/* Provided by DM_motor.c, shared by the old motor modules. */
/* 注意：uint_to_float 的实现实际位于 app_can.c；dm8009_fbdata 也调用它，
 * 但该函数当前无调用者，实际反馈解码走 app_can.c 的 APP_DecodeJoint。 */
extern float uint_to_float(int x_int, float x_min, float x_max, int bits);
#include "motor_encoding.h"

#endif /* _CAN_BSP_H */
