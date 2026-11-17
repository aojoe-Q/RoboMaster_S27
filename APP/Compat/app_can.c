/* app_can.c：A_core 的 CAN 收发与遥控兼容层。
 * 职责：
 *   1) canx_send_data() 把 APP 层电机指令封装成 FDCAN 帧交给 HAL；
 *   2) APP_CAN_RxHandler() 解码 FDCAN1 达妙反馈与 FDCAN2 双板数据；
 *   3) APP_RC_Fallback()/APP_RC_Task() 维护遥控来源优先级并刷新 RCData。
 * 总线：FDCAN1 = 左臂 7 个达妙电机 0x01..0x07（classic CAN 1 Mbit/s）；
 *       FDCAN2 = 双板通信（FD + BRS）；FDCAN3 的抬升电机不在此文件处理。
 * 约定：达妙反馈帧 D0 低 4 位 = 电机 ID，高 4 位 = 状态/错误码；
 *       APP_CAN_RxHandler 由 CAN_Task 在捕获 ISR 校验帧头与长度后调用。 */
#include "can_bsp.h"
#include "bsp_can.h"
#include "fdcan.h"
#include "Remote_Control.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can_health.h"
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
static uint32_t app_board_ch012_tick = 0;  /* 0x212 最后接收时刻 */
static uint32_t app_board_ch3key_tick = 0; /* 0x213 最后接收时刻 */

/* The old can_bsp.h provided this as a static inline helper in every TU.
 * One global definition replaces that; DM_motor.c and app_can.c both use it. */
/* uint_to_float：把达妙反馈里的无符号定点整数还原成物理量。
 * 参数：x_int 原始整数；x_min/x_max 物理量程；bits 位宽（本工程 12 或 16）。
 * 返回：x_min + x_int * (x_max-x_min) / ((1<<bits)-1)。
 * 注意：本函数不做范围检查，调用方必须保证 x_int 落在 [0, (1<<bits)-1]；
 *       与 DM_motor.c 的 float_to_uint_clamped() 近似互逆（量化有取整误差）。 */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span   = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/* DT7 is considered online while the template's remote monitor keeps its
 * online counter above the same threshold used by Remote_Message_Moniter(). */
/* APP_Remote_IsOnline：判断 DT7/SBUS 遥控是否在线。
 * 判据与 Remote_Message_Moniter() 一致：未丢帧且 online_cnt > 0x32（50）。
 * 返回 true 时 APP_RC_Fallback 优先采用 DT7 数据。 */
static bool APP_Remote_IsOnline(void)
{
    return (!remote_ctrl.rc_lost) && (remote_ctrl.online_cnt > 0x32U);
}

/* Dual-board link is considered online only if BOTH required messages are received. */
/* APP_Board_IsOnline：判断双板 CAN 链路是否在线。
 * 判据：0x212 和 0x213 都收到过，且都未超时（各自 100 ms）。
 * 说明：双板分别发送通道 0~2/拨杆（0x212）和通道 3/键盘/鼠标（0x213）；
 *      只有两种报文都在线，才能发布完整的遥控数据。 */
static bool APP_Board_IsOnline(void)
{
    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    return (app_board_ch012_tick != 0U) && (app_board_ch3key_tick != 0U) &&
           ((now_ms - app_board_ch012_tick) < 100U) &&
           ((now_ms - app_board_ch3key_tick) < 100U);
}

/* Copy the decoded DT7 data into the APP's Rcdata_t view. */
/* APP_CopyRemoteToRCData：把 remote_ctrl 的 DT7 数据整体拷贝到 RCData。
 * 调用者：APP_RC_Fallback（10 ms 周期）。
 * 临界区原因：remote_ctrl 由遥控接收 ISR 更新，逐字段拷贝可能读到半新半旧
 * 的快照；关中断保证整段拷贝看到同一时刻的 remote_ctrl。 */
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

/* Build privately, serialize only the short HAL enqueue operation. */
/* canx_send_data：把一帧经典/FD CAN 数据排入指定 FDCAN 的发送 FIFO。
 * 参数：hcan 总线句柄；id 标准帧 ID；data 数据指针；len 1..8 字节。
 * 返回：HAL 状态；参数非法时返回 HAL_ERROR，其余由 HAL 入队结果决定。
 * 注意：FDCAN2（双板）使用 FD 帧 + 波特率切换，其余总线保持 classic；
 *       帧结构在栈上构造，入队由 USER_FDCAN_AddMessageToTxFifoQ 完成。 */
HAL_StatusTypeDef canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id,
                                uint8_t *data, uint32_t len)
{
    FDCAN_TxFrame_TypeDef frame = {0};
    if (!hcan || !data || len == 0U || len > 8U) return HAL_ERROR; // 1..8 字节：本层帧长上限
    frame.hcan = hcan;
    frame.Header.Identifier = id;
    frame.Header.IdType = FDCAN_STANDARD_ID;
    frame.Header.TxFrameType = FDCAN_DATA_FRAME;
    frame.Header.DataLength = len;
    frame.Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    frame.Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    frame.Header.BitRateSwitch = hcan == &hfdcan2 ? FDCAN_BRS_ON : FDCAN_BRS_OFF; // FDCAN2 双板链路打开 BRS
    frame.Header.FDFormat = hcan == &hfdcan2 ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN; // 仅 FDCAN2 用 FD 格式，其余总线 classic
    memcpy(frame.Data, data, len);
    return USER_FDCAN_AddMessageToTxFifoQ(&frame);
}

/* APP_DecodeJoint：解码一帧达妙电机反馈并写回 joint_L_motor[joint]。
 * 参数：joint 0 基关节下标（0..6）；data 8 字节反馈；tick_ms 捕获时刻。
 * 返回：无。
 * 调用者：APP_CAN_RxHandler（FDCAN1 分支）。
 * 注意：D0 低 4 位必须等于 joint+1，否则计 rx_invalid 并丢弃；
 *       joint_L_motor 采用“整体拷贝-本地修改-整体写回”的临界区写法，
 *       避免与读取方（控制任务）交错看到半更新结构。 */
static void APP_DecodeJoint(uint8_t joint, const uint8_t *data, uint32_t tick_ms)
{
    joint_motor_measure_t sample;
    joint_motor_measure_t *motor = &sample;

    if (joint >= 7U) // joint 为 0 基索引，合法范围 0..6
    {
        return;
    }
    if ((data[0] & 0x0FU) != joint + 1U) { can_health[0].rx_invalid++; return; } // D0 低 4 位 = 电机 ID（关节下标 + 1）
    uint8_t status = data[0] >> 4; // 高 4 位：状态/错误码
    if (status > 14U || (status > 1U && status < 8U)) { can_health[0].rx_invalid++; return; } // 合法状态：0/1/8..14
    taskENTER_CRITICAL();
    sample = joint_L_motor[joint];
    taskEXIT_CRITICAL();
    motor->status = status;
	
//	memset(g_Can1g_Can2g_Can2RxData, 0, sizeof(g_Can1g_Can2g_Can2RxData));
//   HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, g_Can1g_Can2g_Can2RxData);

//	joint_L_motor[0].motorid = (g_Can1g_Can2g_Can2RxData[0])&0x0F;		
//	joint_L_motor[0].p_int= (g_Can1g_Can2g_Can2RxData[1]<<8)|g_Can1g_Can2g_Can2RxData[2];
//	joint_L_motor[0].v_int= (g_Can1g_Can2g_Can2RxData[3]<<4)|(g_Can1g_Can2g_Can2RxData[4]>>4);
//	joint_L_motor[0].t_int= ((g_Can1g_Can2g_Can2RxData[4]&0xF)<<8)|g_Can1g_Can2g_Can2RxData[5];
//	joint_L_motor[0].position = uint_to_float( joint_L_motor[0].p_int, P_MIN, P_MAX, 16);
//	joint_L_motor[0].velocity = uint_to_float(joint_L_motor[0].v_int, V_MIN, V_MAX, 12);
//	joint_L_motor[0].torque = uint_to_float(joint_L_motor[0].t_int, T_MIN, T_MAX, 12);
    motor->motorid  = (uint32_t)(data[0] & 0x0FU);
    motor->p_int    = (int)((uint16_t)data[1] << 8 | data[2]); // 位置：D1 高字节、D2 低字节
    motor->v_int    = (int)((uint16_t)data[3] << 4 | (data[4] >> 4)); // 速度：D3 + D4 高 4 位
    motor->t_int    = (int)(((uint16_t)(data[4] & 0x0FU) << 8) | data[5]); // 力矩：D4 低 4 位 + D5
    motor->position = uint_to_float(motor->p_int, P_MIN, P_MAX, 16); // 位置原始值 → rad
    motor->velocity = uint_to_float(motor->v_int, V_MIN, V_MAX, 12); // 速度原始值 → rad/s
    motor->torque   = uint_to_float(motor->t_int, T_MIN, T_MAX, 12); // 力矩原始值 → N·m
    /* Original ISR capture time, unaffected by queue latency. */
    motor->update_tick = tick_ms;
    motor->update_count++; // 反馈计数，供上层判断数据是否新鲜
    taskENTER_CRITICAL();
    joint_L_motor[joint] = sample;
    taskEXIT_CRITICAL();
}

/*
 * Called by CAN_Task after validating header and length in the capture ISR.
 */
/* APP_CAN_RxHandler：CAN 接收分发入口。
 * 参数：hcan 总线；id 标准帧 ID；data 数据；tick_ms 捕获时刻（RTOS tick）。
 * 返回：无。
 * 调用者：CAN_Task（在捕获 ISR 校验帧头/长度之后调用）。
 * 分支：FDCAN1 只处理 0x01..0x07 的达妙反馈；
 *       FDCAN2 处理双板 0x212/0x213、0x208/0x209/0x210、0x207。 */
void APP_CAN_RxHandler(FDCAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint32_t tick_ms)
{
    if ((hcan == NULL) || (data == NULL))
    {
        return;
    }

    if (hcan->Instance == FDCAN1) // FDCAN1 只处理左臂 0x01..0x07
    {
        /* Left arm DM motor feedback: IDs 0x01..0x07. */
        if ((id >= (uint32_t)CAN_DM_L_J1_ID) && (id <= (uint32_t)CAN_DM_L_GRIP_ID))
        {
            APP_DecodeJoint((uint8_t)(id - 1U), data, tick_ms); // 帧 ID 1..7 对应关节下标 0..6
        }
    }
    /* FDCAN2：双板/自定义控制器数据，整段临界区保护。
     * 0x212：ch0..ch2、s[1]/s[0]；0x213：鼠标按键/键盘/ch3/mouse.x；
     * 0x208/0x209/0x210：4 路 float 两两一帧；0x207：CC_Key_grip/J4。
     * 多帧共同拼 app_board_rc_data / CC_data，关中断避免读到半更新。 */
    else if (hcan->Instance == FDCAN2)
    {
        taskENTER_CRITICAL();
        /* Dual-board remote data. */
        if (id == (uint32_t)CAN_Board_ID_1)  /* 0x212 */
        {
            app_board_rc_data.rc.ch[0] = (int16_t)((uint16_t)data[0] << 8 | data[1]);
            app_board_rc_data.rc.ch[1] = (int16_t)((uint16_t)data[2] << 8 | data[3]);
            app_board_rc_data.rc.ch[2] = (int16_t)((uint16_t)data[4] << 8 | data[5]);
            app_board_rc_data.rc.s[1]  = (char)data[6];
            app_board_rc_data.rc.s[0]  = (char)data[7];
            app_board_ch012_tick = tick_ms;  /* 独立记录 0x212 时刻 */
        }
        else if (id == (uint32_t)CAN_Board_ID_2)  /* 0x213 */
        {
            app_board_rc_data.mouse.press_l = data[0];
            app_board_rc_data.mouse.press_r = data[1];
            app_board_rc_data.key.v         = (uint16_t)((uint16_t)data[2] << 8 | data[3]);
            app_board_rc_data.rc.ch[3]      = (int16_t)((uint16_t)data[4] << 8 | data[5]);
            app_board_rc_data.mouse.x       = (int16_t)((uint16_t)data[6] << 8 | data[7]);
            app_board_ch3key_tick           = tick_ms;  /* 独立记录 0x213 时刻 */
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
        taskEXIT_CRITICAL();
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
/* APP_RC_Fallback：按优先级刷新 RCData。
 * 顺序：DT7 在线 → 用 remote_ctrl；否则双板在线 → 用影子缓冲；
 *       两者都离线 → 拷贝已清零的 remote_ctrl（ARM_POWERLESS 安全态）。
 * 调用者：APP_RC_Task，每 10 ms 一次；双板帧只写影子缓冲，
 * 因此永远不会覆盖在线的 DT7 数据。 */
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

/* APP_RC_Task：遥控刷新任务。
 * 参数：argument 未使用（FreeRTOS 任务签名要求）。
 * 周期：osDelay(10)，即 10 ms 刷新一次 RCData；
 * 注意：本任务只做数据选择/拷贝，不做控制运算。 */
void APP_RC_Task(void const *argument)
{
    (void)argument;

    for (;;)
    {
        APP_RC_Fallback();
        osDelay(10); // 10 ms 刷新一次遥控数据
    }
}
