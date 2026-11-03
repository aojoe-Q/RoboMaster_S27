#include "can_bsp.h"
#include "bsp_can.h"
#include "fdcan.h"
#include "Remote_Control.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include "arm_output.h"
#include "arm_input.h"
#include "master_link.h"

/* 关节反馈和影子板数据都放在这里，APP 层源码无需改动。
 * joint_L_motor 本身由 Arm_L_task.c 定义（与 S27_ARM_H7 一致）。 */
Rcdata_t              RCData;
A_board_CC_data_t     CC_data;

/* 影子板的 CAN 数据解码到这份影子缓冲区；RCData 在 APP_RC_Task 中选择：
 * DT7 优先，这份缓冲是备份。 */
static Rcdata_t       app_board_rc_data;

/* 影子板两个分帧各自的时间戳与序号（0 表示从未收到）。 */
static uint32_t app_board_switch_tick,app_board_key_tick;
static uint32_t app_board_switch_seq,app_board_key_seq;
volatile uint32_t arm_dt7_tick,arm_dt7_seq;
static ArmInputSnapshot arm_input;


/* 旧版 can_bsp.h 在每个编译单元里提供这个内联函数；
 * 这里改成一个全局定义，DM_motor.c 与 app_can.c 共用。 */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span   = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/* 只要模板的遥控监测器把在线计数保持在 Remote_Message_Moniter() 使用的
 * 同一阈值以上，就认为 DT7 在线。 */
static bool APP_Remote_IsOnline(void)
{
    return (!remote_ctrl.rc_lost) && (remote_ctrl.online_cnt > 0x32U);
}

/* 影子板链路上次收到 CAN 帧后 100 ms 内视为在线。 */
static bool APP_Board_IsOnline(void)
{
    uint32_t now=HAL_GetTick();
    return app_board_switch_seq && app_board_key_seq &&
        (uint32_t)(now-app_board_switch_tick)<100 &&
        (uint32_t)(now-app_board_key_tick)<100;
}

/* 把解码好的 DT7 数据复制到 APP 的 Rcdata_t 视图。 */
static void APP_CopyRemoteToRCData(void)
{
    /* 与 Remote_Message_Moniter()/SBUS 中断看到的快照保持一致。 */
    taskENTER_CRITICAL();

    RCData.rc.ch[0] = remote_ctrl.rc.ch[0];
    RCData.rc.ch[1] = remote_ctrl.rc.ch[1];
    RCData.rc.ch[2] = remote_ctrl.rc.ch[2];
    RCData.rc.ch[3] = remote_ctrl.rc.ch[3];
    RCData.rc.ch[4] = remote_ctrl.rc.ch[4];
    /* remote_ctrl 与 Rcdata_t 采用同一约定：
     * [0] = 右拨杆，[1] = 左拨杆。 */
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

/* HAL 的 DLC 编码：0..8 字节时 DLC 值就等于字节数。 */
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

    /* 模板的 TxFrame 对象被多个 APP 任务共用；
     * 把头部/数据更新和 HAL 入队放在同一个短临界区里。 */
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
    if ((data[0]&0x0FU)!=(joint+1U)) return;
    motor = &joint_L_motor[joint];

    motor->motorid  = (uint32_t)(data[0] & 0x0FU);
    motor->p_int    = (int)((uint16_t)data[1] << 8 | data[2]);
    motor->v_int    = (int)((uint16_t)data[3] << 4 | (data[4] >> 4));
    motor->t_int    = (int)(((uint16_t)(data[4] & 0x0FU) << 8) | data[5]);
    motor->position = uint_to_float(motor->p_int, P_MIN, P_MAX, 16);
    motor->velocity = uint_to_float(motor->v_int, V_MIN, V_MAX, 12);
    motor->torque   = uint_to_float(motor->t_int, T_MIN, T_MAX, 12);
    ArmOutput_Feedback(joint,motor->position,data[0]>>4,HAL_GetTick());
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
        /* 左臂 DM 电机反馈：ID 0x01..0x07。 */
        if ((id >= (uint32_t)CAN_DM_L_J1_ID) && (id <= (uint32_t)CAN_DM_L_GRIP_ID))
        {
            APP_DecodeJoint((uint8_t)(id - 1U), data);
        }
    }
    else if (hcan->Instance == FDCAN2)
    {
        /* 影子板遥控数据，布局与 S27_ARM_H7 的 can_bsp.c 相同。 */
        if (id == (uint32_t)CAN_Board_ID_1)
        {
            app_board_rc_data.rc.ch[0] = (int16_t)((uint16_t)data[0] << 8 | data[1]);
            app_board_rc_data.rc.ch[1] = (int16_t)((uint16_t)data[2] << 8 | data[3]);
            app_board_rc_data.rc.ch[2] = (int16_t)((uint16_t)data[4] << 8 | data[5]);
            app_board_rc_data.rc.s[1]  = (char)data[6];
            app_board_rc_data.rc.s[0]  = (char)data[7];
            app_board_switch_tick = HAL_GetTick(); app_board_switch_seq++;
        }
        else if (id == (uint32_t)CAN_Board_ID_2)
        {
            app_board_rc_data.mouse.press_l = data[0];
            app_board_rc_data.mouse.press_r = data[1];
            app_board_rc_data.key.v         = (uint16_t)((uint16_t)data[2] << 8 | data[3]);
            app_board_rc_data.rc.ch[3]      = (int16_t)((uint16_t)data[4] << 8 | data[5]);
            app_board_rc_data.mouse.x       = (int16_t)((uint16_t)data[6] << 8 | data[7]);
            app_board_key_tick = HAL_GetTick(); app_board_key_seq++;
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
    ImageKeyboardSnapshot image_keyboard;
    bool image_key_ok;
    uint32_t p=__get_PRIMASK(); __disable_irq();
    memset(&arm_input,0,sizeof(arm_input));
    /* 图传键盘是一条明确的替代来源，绝不与本地键盘按位或合并；
       它有自己的时间戳和自己的序号。 */
    image_key_ok=ImageKeyboard_Read(&image_keyboard) &&
        (uint32_t)(HAL_GetTick()-image_keyboard.rx_tick)<100;
    if(APP_Remote_IsOnline() && arm_dt7_seq && (uint32_t)(HAL_GetTick()-arm_dt7_tick)<100) {
        APP_CopyRemoteToRCData();
        arm_input.source=1; arm_input.switches_tick=arm_dt7_tick;
        arm_input.keyboard_tick=arm_dt7_tick; arm_input.seq=arm_dt7_seq;
    } else if(APP_Board_IsOnline()) {
        memcpy(&RCData,&app_board_rc_data,sizeof(RCData));
        arm_input.source=2; arm_input.switches_tick=app_board_switch_tick;
        arm_input.keyboard_tick=app_board_key_tick; arm_input.seq=app_board_switch_seq;
    } else {
        memset(&RCData,0,sizeof(RCData));
    }
    arm_input.s[0]=RCData.rc.s[0]; arm_input.s[1]=RCData.rc.s[1];
    if(image_key_ok) {
        arm_input.key=(uint16_t)(image_keyboard.payload[8]|((uint16_t)image_keyboard.payload[9]<<8));
        arm_input.keyboard_source=3;
        arm_input.keyboard_tick=image_keyboard.rx_tick;
        arm_input.seq=image_keyboard.seq;
    } else {
        arm_input.key=RCData.key.v;
        arm_input.keyboard_source=(uint8_t)arm_input.source;
    }
    arm_input.online=arm_input.source!=0 &&
        arm_input.s[0]>=1 && arm_input.s[0]<=3 && arm_input.s[1]>=1 && arm_input.s[1]<=3;
    __set_PRIMASK(p);
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

void ArmInput_Read(ArmInputSnapshot *out)
{
    uint32_t p=__get_PRIMASK(); __disable_irq(); *out=arm_input; __set_PRIMASK(p);
    if((uint32_t)(HAL_GetTick()-out->switches_tick)>=100 ||
       (uint32_t)(HAL_GetTick()-out->keyboard_tick)>=100) out->online=false;
}
