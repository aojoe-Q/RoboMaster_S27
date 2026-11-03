#include "can_bsp.h"
#include "bsp_can.h"
#include "fdcan.h"
#include "Remote_Control.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* Joint feedback and board data live here so the APP sources stay unchanged.
 * joint_L_motor itself is defined by Arm_L_task.c (as in S27_ARM_H7). */
Rcdata_t              RCData;
A_board_CC_data_t     CC_data;

/* Dual-board CAN data is decoded into this shadow buffer. RCData itself is
 * selected in APP_RC_Task: DT7 has priority, this buffer is the backup. */
static Rcdata_t       app_board_rc_data;

/* Last time a dual-board frame was received (0 = never). */
static uint32_t app_board_last_tick = 0;

/* The old can_bsp.h provided this as a static inline helper in every TU.
 * One global definition replaces that; DM_motor.c and app_can.c both use it. */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span   = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/* DT7 is considered online while the template's remote monitor keeps its
 * online counter above the same threshold used by Remote_Message_Moniter(). */
static bool APP_Remote_IsOnline(void)
{
    return (!remote_ctrl.rc_lost) && (remote_ctrl.online_cnt > 0x32U);
}

/* Dual-board link is considered online for 100 ms after its last CAN frame. */
static bool APP_Board_IsOnline(void)
{
    return (app_board_last_tick != 0U) &&
           ((HAL_GetTick() - app_board_last_tick) < 100U);
}

/* Copy the decoded DT7 data into the APP's Rcdata_t view. */
static void APP_CopyRemoteToRCData(void)
{
    /* Keep the snapshot consistent with Remote_Message_Moniter()/SBUS ISR. */
    taskENTER_CRITICAL();

    RCData.rc.ch[0] = remote_ctrl.rc.ch[0];
    RCData.rc.ch[1] = remote_ctrl.rc.ch[1];
    RCData.rc.ch[2] = remote_ctrl.rc.ch[2];
    RCData.rc.ch[3] = remote_ctrl.rc.ch[3];
    RCData.rc.ch[4] = remote_ctrl.rc.ch[4];
    /* remote_ctrl and Rcdata_t share the same convention:
     * [0] = right switch, [1] = left switch. */
    RCData.rc.s[0]  = (char)remote_ctrl.rc.s[0];
    RCData.rc.s[1]  = (char)remote_ctrl.rc.s[1];
    RCData.mouse.x  = remote_ctrl.mouse.x;
    RCData.mouse.y  = remote_ctrl.mouse.y;
    RCData.mouse.z  = remote_ctrl.mouse.z;
    RCData.mouse.press_l = remote_ctrl.mouse.press_l;
    RCData.mouse.press_r = remote_ctrl.mouse.press_r;
    RCData.key.v    = remote_ctrl.key.v;

    taskEXIT_CRITICAL();
}

static FDCAN_TxFrame_TypeDef *APP_GetTxFrame(FDCAN_HandleTypeDef *hcan)
{
    if (hcan == NULL)
    {
        return NULL;
    }

    if (hcan->Instance == FDCAN1)
    {
        return &FDCAN1_TxFrame;
    }
    if (hcan->Instance == FDCAN2)
    {
        return &FDCAN2_TxFrame;
    }
    if (hcan->Instance == FDCAN3)
    {
        return &FDCAN3_TxFrame;
    }
    return NULL;
}

/* HAL DLC encoding: for 0..8 bytes the DLC value equals the byte count. */
static uint32_t APP_DlcFromLen(uint32_t len)
{
    if (len <= 8U)
    {
        return len;
    }
    switch (len)
    {
        case 12U: return FDCAN_DLC_BYTES_12;
        case 16U: return FDCAN_DLC_BYTES_16;
        case 20U: return FDCAN_DLC_BYTES_20;
        case 24U: return FDCAN_DLC_BYTES_24;
        case 32U: return FDCAN_DLC_BYTES_32;
        case 48U: return FDCAN_DLC_BYTES_48;
        case 64U: return FDCAN_DLC_BYTES_64;
        default:  return FDCAN_DLC_BYTES_8;
    }
}

/*
 * Drop-in replacement for the old S27_ARM_H7 canx_send_data().
 * It re-uses the COD template TxFrame objects, so the FDCAN1/2/3 bit-rate
 * and frame format settings come from bsp_can.c instead of being duplicated.
 */
void canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id,
                    uint8_t *data, uint32_t len)
{
    FDCAN_TxFrame_TypeDef *frame = APP_GetTxFrame(hcan);

    if ((frame == NULL) || (data == NULL) || (len == 0U))
    {
        return;
    }
    if (len > 8U)
    {
        len = 8U;
    }

    /* The template TxFrame objects are shared by the APP tasks; keep the
     * header/data update and the HAL push in one short critical section. */
    taskENTER_CRITICAL();

    frame->Header.Identifier          = id;
    frame->Header.IdType              = FDCAN_STANDARD_ID;
    frame->Header.TxFrameType         = FDCAN_DATA_FRAME;
    frame->Header.DataLength          = APP_DlcFromLen(len);
    frame->Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    frame->Header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    frame->Header.MessageMarker       = 0U;

    memset(frame->Data, 0, sizeof(frame->Data));
    memcpy(frame->Data, data, len);

    USER_FDCAN_AddMessageToTxFifoQ(frame);

    taskEXIT_CRITICAL();
}

static void APP_DecodeJoint(uint8_t joint, const uint8_t *data)
{
    joint_motor_measure_t *motor;

    if (joint >= 7U)
    {
        return;
    }
    motor = &joint_L_motor[joint];

    motor->motorid  = (uint32_t)(data[0] & 0x0FU);
    motor->p_int    = (int)((uint16_t)data[1] << 8 | data[2]);
    motor->v_int    = (int)((uint16_t)data[3] << 4 | (data[4] >> 4));
    motor->t_int    = (int)(((uint16_t)(data[4] & 0x0FU) << 8) | data[5]);
    motor->position = uint_to_float(motor->p_int, P_MIN, P_MAX, 16);
    motor->velocity = uint_to_float(motor->v_int, V_MIN, V_MAX, 12);
    motor->torque   = uint_to_float(motor->t_int, T_MIN, T_MAX, 12);
    /* FDCAN RX interrupt (priority 5, FreeRTOS-safe). */
    motor->update_tick = (uint32_t)xTaskGetTickCountFromISR();
    motor->update_count++;
}

/*
 * Called from the existing bsp_can.c RX callbacks. The template callbacks keep
 * their own motor decoding; this hook adds the APP arm / dual-board decoding.
 */
void APP_CAN_RxHandler(FDCAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data)
{
    if ((hcan == NULL) || (data == NULL))
    {
        return;
    }

    if (hcan->Instance == FDCAN1)
    {
        /* Left arm DM motor feedback: IDs 0x01..0x07. */
        if ((id >= (uint32_t)CAN_DM_L_J1_ID) && (id <= (uint32_t)CAN_DM_L_GRIP_ID))
        {
            APP_DecodeJoint((uint8_t)(id - 1U), data);
        }
    }
    else if (hcan->Instance == FDCAN2)
    {
        /* Dual-board remote data, same layout as S27_ARM_H7 can_bsp.c. */
        if (id == (uint32_t)CAN_Board_ID_1)
        {
            app_board_rc_data.rc.ch[0] = (int16_t)((uint16_t)data[0] << 8 | data[1]);
            app_board_rc_data.rc.ch[1] = (int16_t)((uint16_t)data[2] << 8 | data[3]);
            app_board_rc_data.rc.ch[2] = (int16_t)((uint16_t)data[4] << 8 | data[5]);
            app_board_rc_data.rc.s[1]  = (char)data[6];
            app_board_rc_data.rc.s[0]  = (char)data[7];
            app_board_last_tick = HAL_GetTick();
        }
        else if (id == (uint32_t)CAN_Board_ID_2)
        {
            app_board_rc_data.mouse.press_l = data[0];
            app_board_rc_data.mouse.press_r = data[1];
            app_board_rc_data.key.v         = (uint16_t)((uint16_t)data[2] << 8 | data[3]);
            app_board_rc_data.rc.ch[3]      = (int16_t)((uint16_t)data[4] << 8 | data[5]);
            app_board_rc_data.mouse.x       = (int16_t)((uint16_t)data[6] << 8 | data[7]);
            app_board_last_tick            = HAL_GetTick();
        }
        else if (id == (uint32_t)CAN_Board__float_ID_1)
        {
            memcpy(&CC_data.DM_CC_1_receive, data, 4);
            memcpy(&CC_data.DM_CC_2_receive, data + 4, 4);
        }
        else if (id == (uint32_t)CAN_Board__float_ID_2)
        {
            memcpy(&CC_data.DM_CC_3_receive, data, 4);
            memcpy(&CC_data.DM_CC_4_receive, data + 4, 4);
        }
        else if (id == (uint32_t)CAN_Board__float_ID_3)
        {
            memcpy(&CC_data.DJ_CC_1_receive, data, 4);
            memcpy(&CC_data.DJ_CC_2_receive, data + 4, 4);
        }
        else if (id == (uint32_t)CAN_Board__float_ID_4)
        {
            CC_data.CC_Key_grip = data[0];
            CC_data.CC_Key_J4   = data[1];
        }
    }
}

/*
 * Remote priority:
 *   1) DT7/SBUS online  -> RCData = remote_ctrl
 *   2) DT7 offline, dual board online -> RCData = shadow board data
 *   3) both offline -> copy the zeroed remote_ctrl (safe/powerless state)
 *
 * APP_RC_Task calls this every 10 ms. The dual-board CAN frames only fill the
 * shadow buffer, so they can never overwrite an online DT7.
 */
void APP_RC_Fallback(void)
{
#if APP_RC_FALLBACK_ENABLE
    if (APP_Remote_IsOnline())
    {
        APP_CopyRemoteToRCData();
    }
    else if (APP_Board_IsOnline())
    {
        taskENTER_CRITICAL();
        memcpy(&RCData, &app_board_rc_data, sizeof(RCData));
        taskEXIT_CRITICAL();
    }
    else
    {
        /* Remote_Message_Moniter() has cleared remote_ctrl, so this also
         * clears RCData and makes GetArmMode() fall back to ARM_POWERLESS. */
        APP_CopyRemoteToRCData();
    }
#endif
}

void APP_RC_Task(void const *argument)
{
    (void)argument;

    for (;;)
    {
        APP_RC_Fallback();
        osDelay(10);
    }
}
