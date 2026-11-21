#include "arm_output.h"
#include "arm_motion.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Arm_L_task.h"
#include "Arm_USB_task.h"
#include "DM_motor.h"
#include "DJI_motor.h"
#include "can_bsp.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "uart_bsp.h"
#include "VT13_rc_ctrl.h"
#include "DT7_DR16.h"
#include "LK_motor.h"
#include "referee_usart_task.h"
#include "math.h"
/* rc_deadband_limit(input, output, dealine)：摇杆去死区——|input| 不超过 dealine 时
 * output 置 0，否则原样输出；用来滤掉摇杆回中附近的小抖振。 */
#define rc_deadband_limit(input, output, dealine)    \
  {                                                  \
    if ((input) > (dealine) || (input) < -(dealine)) \
    {                                                \
      (output) = (input);                            \
    }                                                \
    else                                             \
    {                                                \
      (output) = 0;                                  \
    }                                                \
  }
  
/* 三路 FDCAN 句柄：本文件只用 hfdcan1（机械臂 7 个电机挂在这条总线上）。 */
  extern FDCAN_HandleTypeDef hfdcan1;
  extern FDCAN_HandleTypeDef hfdcan2;
  extern FDCAN_HandleTypeDef hfdcan3;
  
  extern DT7_RC_ctrl_t DT7_rc_ctrl;    //DT7
  extern VT13_RC_ctrl_t VT13_rc_ctrl;  //VT13

/* 7 个关节的斜坡状态量（ramp_function_source_t，定义见 user_lib.h）
 * 用法：arm_ramp_calc(&x, 每拍步长, 目标值) 之后取 x.out 作为本拍目标；
 *       各模式共用同一组状态量，所以模式切换时要同步 .out，避免从旧目标起跳。 */
  ramp_function_source_t J1_L_position_set;      //DM_4340p  
  ramp_function_source_t J2_L_position_set;	     //DM_6248p
  ramp_function_source_t J3_L_position_set;      //DM_6248p
  ramp_function_source_t J4_L_position_set;      //DM_4310P
  ramp_function_source_t J5_L_position_set;      //DM_4310P
  ramp_function_source_t J6_L_position_set;      //DM_4310P
  ramp_function_source_t grip_L_position_set;    //DM_4310
  
/* 备用的一组斜坡状态量：全文件没有任何读写，保留占位。 */
  ramp_function_source_t J1_L_top_set;      //DM_4340p  
  ramp_function_source_t J2_L_top_set;	     //DM_J6248p
  ramp_function_source_t J3_L_top_set;      //DM_6248p
  ramp_function_source_t J4_L_top_set;      //DM_4310P
  ramp_function_source_t J5_L_top_set;      //DM_4310P
  ramp_function_source_t J6_L_top_set;      //DM_4310P
  ramp_function_source_t grip_L_top_set;    //DM_4310
  
/* J4 单轴按键微调用的两条斜坡：只在 Arm_L_Init 里初始化过，之后未使用。 */
  ramp_function_source_t J4_L_key2_1_set;     
  ramp_function_source_t J4_L_key2_0_set;    
  
/* ---------------- 全局运行状态量 ---------------- */
  ButtonCounter left_button_btn;  
  ButtonCounter right_button_btn;
  ArmData_t ArmData; // 全部关节的控制量与当前模式（输出层也从这里读）
  volatile uint8_t Arm_L_Ready = 0;  /* set when Arm_L_Init() has finished */
  volatile ArmGripZeroDiagnostics arm_grip_zero_diag;
  volatile ArmHomeDiagnostics arm_home_diag;
  RC_contorl_t RC_contorl; // VT13 遥控器封装；当前实际使用的是 RCData，见 GetArmMode()
  
  joint_motor_measure_t joint_L_motor[7]; // 7 个电机的反馈，下标 0..6 ↔ ID 1..7，由 CAN 接收路径写入
  
  
/* 上电默认姿态(rad)：用作回零/模式切换的初始基准。 */

float J1_L_initial_posture = 0.0,J2_L_initial_posture = 0.0f,
        J3_L_initial_posture = 0.0f,J4_L_initial_posture = 0.0f,
        J5_L_initial_posture = 0.0f,J6_L_initial_posture = 0.0f,
        grip_L_initial_posture = 0.0f;
/* 目标值中间变量：各模式函数先算到这里，限位/斜坡处理后再写回 ArmData.*.send_set。 */
  fp32 J1_send_set, J2_send_set, J3_send_set, J4_send_set, J5_send_set, J6_send_set, Grip_send_set,
       J1_set_initial_data, J2_set_initial_data, J3_set_initial_data, J4_set_initial_data, J5_set_initial_data, J6_set_initial_data, Grip_set_initial_data;
/* 初始化标志：J1..J7_init_flag 在 Arm_L_Init 的回零循环里当循环条件。 */
  fp32 grip_pawl_flag = 1,J1_J2_J3_J4_J5_J6_J7_init_flag = 1;
  fp32 slackawl_flag = 1; // 当前无引用的历史变量
  uint8_t left_button_stop_state, right_button_stop_state;
  uint32_t left_button_press_count, right_button_press_count;
  fp32 limit_data5,limit_data6; // J5/J6 自定义控制器的限位中间量（只在注释掉的旧代码里出现）
  float Init_position; // 无引用的历史变量
  float Last_position;
/* 各关节斜坡步长(rad/拍)：注意类型是 double，一拍 = 一次模式函数调用。 */
  double J1_ramp=0,J2_ramp=0,J3_ramp=0,J4_ramp=0,J5_ramp=0,J6_ramp=0,Grip_ramp=0; // 每拍最大转角，各模式函数里会按需重新赋值
  
  int i=1;

static uint8_t grip_torque_mode;
/* Debounced physical DOWN/DOWN selection; retained across invalid RC input. */
static uint8_t retry_powerless_selected;

/* 把 7 个关节放进数组，后面的公共处理就不必重复写 J1、J2……Grip。 */
static DM_contorl_t *const arm_joint_control[7] = 
{
    &ArmData.J1, &ArmData.J2, &ArmData.J3, &ArmData.J4,
    &ArmData.J5, &ArmData.J6, &ArmData.Grip
};
static ramp_function_source_t *const arm_joint_ramp[7] = 
{
    &J1_L_position_set, &J2_L_position_set, &J3_L_position_set,
    &J4_L_position_set, &J5_L_position_set, &J6_L_position_set,
    &grip_L_position_set
};

/* 夹爪力矩混合系数 */
static float s_grip_blend;

/* 待发布的快照 */
static Arm_Control_Snapshot arm_pending_command;

/* 读取夹爪反馈 */
static joint_motor_measure_t Arm_ReadGripFeedback(void)
{
    joint_motor_measure_t feedback;
    taskENTER_CRITICAL();
    feedback = joint_L_motor[6];
    taskEXIT_CRITICAL();
    return feedback;
}

/* 保存电机零点 */
static void Arm_SaveMotorZero(void)
{
    for (uint8_t id = 1U; id <= 6U; ++id) 
    {
        save_motor_zero(&hfdcan1, id, 0U);
        osDelay(5);
    }
}

/* 同步反馈位置到 ArmData */
static void Arm_SyncInitialPosition(void)
{
    taskENTER_CRITICAL();
    for (uint8_t i = 0U; i < 7U; ++i) 
    {
        arm_joint_control[i]->position = joint_L_motor[i].position;
        arm_joint_control[i]->send_set = arm_joint_control[i]->position;
        arm_joint_control[i]->Initial_Total_position = arm_joint_control[i]->position;
    }
    taskEXIT_CRITICAL();
}

/* 每拍更新实测位置 */
static void Arm_UpdateMeasuredPositions(void)
{
    taskENTER_CRITICAL();
    for (uint8_t i = 0U; i < 7U; ++i) 
    {
        arm_joint_control[i]->position = joint_L_motor[i].position;
    }
    taskEXIT_CRITICAL();
}

/* 等待并收集 7 个电机的反馈数据
 * 参数：timeout_ms  最多等待时间（ms）
 * 返回：1 = 全部 7 个电机反馈到达，0 = 超时或部分缺失
 * 说明：轮询 joint_L_motor[] 的时间戳，直到最后更新时间都在本次等待期间内。
 */
static uint8_t Arm_WaitMotorFeedback(uint32_t timeout_ms)
{
    uint32_t start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint8_t feedback_ready[7] = {0};  // 标记各电机反馈是否已到达

    while (xTaskGetTickCount() * portTICK_PERIOD_MS - start_ms < timeout_ms)
     {
        uint8_t all_ready = 1U;
        for (uint8_t i = 0U; i < 7U; ++i) {
            uint32_t update_delay = xTaskGetTickCount() * portTICK_PERIOD_MS - joint_L_motor[i].update_tick;
            /* 反馈在最近 100 ms 内更新过，则认为有效 */
            if (update_delay <= 100U) 
            {
                feedback_ready[i] = 1U;
            } else 
            {
                all_ready = 0U;
            }
        }
        if (all_ready) return 1U;  // 全部就绪
        osDelay(20);
    }
    return 0U;  // 超时：至少有一个电机反馈超时
}

/* 从 CAN 反馈同步所有 7 轴的当前位置到 ArmData
 * 职责：原样拷贝 joint_L_motor[] -> ArmData.Jx.position（临界区保护）
 */
static void Arm_CollectMotorPositions(void)
{
    taskENTER_CRITICAL();
    for (uint8_t i = 0U; i < 7U; ++i) 
    {
        arm_joint_control[i]->position = joint_L_motor[i].position;
    }
    taskEXIT_CRITICAL();
}

/* 初始化 7 轴的目标位置为当前实测值（防止上电瞬间位置阶跃）
 * 职责：send_set / Initial_Total_position 都对齐到实测的 position
 */
static void Arm_InitTargetPositions(void)
{
    taskENTER_CRITICAL();
    for (uint8_t i = 0U; i < 7U; ++i) 
    {
        arm_joint_control[i]->send_set = arm_joint_control[i]->position;
        arm_joint_control[i]->Initial_Total_position = arm_joint_control[i]->position;
    }
    taskEXIT_CRITICAL();
}

/* 让 7 个斜坡从上电时的实测位置开始。 */
static void Arm_InitPositionRamps(void)
{
    for (uint8_t i = 0U; i < 7U; ++i)
    {
        arm_data_ramp_init(arm_joint_ramp[i], 1.0f,
                           arm_joint_control[i]->Initial_Total_position);
    }

    /* J4 的按键微调斜坡虽然暂未使用，仍保留原来的初始化。 */
    arm_data_ramp_init(&J4_L_key2_1_set, 1.0f, ArmData.J4.Initial_Total_position);
    arm_data_ramp_init(&J4_L_key2_0_set, 1.0f, ArmData.J4.Initial_Total_position);
}

/* 初始化正常控制所需的 PD 增益。 */
//喵喵
static void Arm_InitJointGains(void)
{
    const fp32 kp[7] = 
    {
        J1_PID_KP, J2_PID_KP, J3_PID_KP, J4_PID_KP,
        J5_PID_KP, J6_PID_KP, Grip_PID_KP
    };
    const fp32 kd[7] = 
    {
        J1_PID_KD, J2_PID_KD, J3_PID_KD, J4_PID_KD,
        J5_PID_KD, J6_PID_KD, Grip_PID_KD
    };

    for (uint8_t i = 0U; i < 7U; ++i)
    {
        arm_joint_control[i]->Kp = kp[i];
        arm_joint_control[i]->Kd = kd[i];
    }
}

/* 回零完成后清除遥控和按键累加量。 */
static void Arm_ClearControlOffsets(void)
{
    for (uint8_t i = 0U; i < 7U; ++i)
    {
        arm_joint_control[i]->RC_Sum_Value = 0.0f;
        arm_joint_control[i]->Key_Control_Value = 0.0f;
    }
}

/* 回零完成后让所有斜坡从零点重新开始。 */
static void Arm_ResetPositionRamps(void)
{
    for (uint8_t i = 0U; i < 7U; ++i)
    {
        arm_data_ramp_init(arm_joint_ramp[i], 1.0f, 0.0f);
    }
}
/* 模式变更处理 */

/* 准备快照并发布 */
static void Arm_PrepareAndPublish(void)
{
    static uint32_t last_ms;
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    float dt_ms = (float)(now - last_ms);
    last_ms = now;

    /* 更新夹爪混合系数 */
    float step = fminf(dt_ms * 0.01f, 0.16f);
    uint8_t use_grip_torque = Arm_L_Ready && grip_torque_mode && ArmData.ArmMode != ARM_MODE_PC_CONTROL;
    if (use_grip_torque)
        s_grip_blend = fminf(1.0f, s_grip_blend + step);
    else
        s_grip_blend = fmaxf(0.0f, s_grip_blend - step);
    if (arm_output_diag.bus_inhibit || arm_output_diag.startup_failed)
        s_grip_blend = 0.0f;

    /* 收集快照 */
    Arm_Control_Snapshot cmd = {0};
    for (uint8_t i = 0U; i < 7U; ++i)
 {
        cmd.pos[i] = arm_joint_control[i]->send_set;
        cmd.kp[i] = arm_joint_control[i]->Kp;
        cmd.kd[i] = arm_joint_control[i]->Kd;
        cmd.torque[i] = arm_joint_control[i]->torque;
    }
    cmd.tick_ms = now;

    /* 叠加夹爪混合：当开关开启时，根据 blend 系数混合 Kp/Kd 和力矩 */
    if (use_grip_torque && !arm_output_diag.startup_failed)
    {
        float blend = s_grip_blend;
        cmd.kp[6] *= 1.0f - blend;
        cmd.kd[6] *= 1.0f - blend;
        cmd.torque[6] = cmd.torque[6] * (1.0f - blend) + 0.2f * blend;
    }

    /* 发布 */
    arm_pending_command = cmd;
    Arm_Control_Publish(&arm_pending_command);
}

/* 快速循环中发布上一次的快照（不重新计算） */
static void Arm_PublishCurrent(void)
{
    Arm_Control_Publish(&arm_pending_command);
}

/* Arm_L_Task() —— 机械臂控制任务主循环（osPriorityHigh，8 ms 周期 ≈ 125 Hz）
 * Arm_L_Init() 只在开始时执行一次，之后循环执行“更新输入 -> 模式 -> 发布”。
 *       data_updata() 更新实测并调用 GetArmMode()；GetArmMode() 内部完成拨杆消抖、
 *       安全覆盖和模式进入动作 -> switch 分派各模式函数 -> 准备快照 -> 发布 ->
 *       有效拨杆切到双下时，在零输出发布后请求重试 -> vTaskDelayUntil 保持周期
 * 注意：本函数里的每个延时都会推迟整条控制链路，阻塞式流程（如
 *       Arm_return_initial_position）不能直接在主循环里调用。
 */
void Arm_L_Task()
{
    Arm_L_Init();  // 初始化完成后，Arm_L_Ready = 1

    uint8_t prev_retry_selection = retry_powerless_selected;
    TickType_t next = xTaskGetTickCount();

    while (1) {
        data_updata();

        uint8_t request_retry = Arm_L_Ready && retry_powerless_selected &&
                                !prev_retry_selection;
        prev_retry_selection = retry_powerless_selected;

        if (!Arm_L_Ready || arm_output_diag.bus_inhibit) 
        {
            ArmData.ArmMode = ARM_POWERLESS;
            grip_torque_mode = 0U;
        }

        /* 模式分派 */
        switch (ArmData.ArmMode) 
        {
			case TAKE_POSE_1:
                TAKE_pose_1();
			break;
			case TAKE_POSE_2:
                TAKE_pose_2();
			break;
            case ARM_RC:
                ARM_rc();
                break;
            case ARM_MODE_PC_CONTROL:
                Arm_PC_Control();
                break;
            case ARM_POWERLESS:
                Arm_Powerless();
                break;
            case ARM_RESET:
                ARM_reset();
                break;
            default:
                Arm_Powerless();
                break;
        }

        Arm_PrepareAndPublish();

        /* Publish zero gains/torque before the higher-priority output task
         * can consume this explicit operator request. */
        if (request_retry)
     {
            Arm_Output_RequestRetry();
        }

        vTaskDelayUntil(&next, pdMS_TO_TICKS(8));
    }
}
	
  

/* ======================= 夹爪堵转标零 ======================= */
/* 上电后夹爪的零点未知。用恒定力矩朝 GRIP_ZERO_DIR 方向顶（不是位置环）：
   位置连续 5 拍（约 100 ms）不变，就认为顶到机械死点，把该位置存成零点（0xFE），然后立刻撤掉力矩。
   不要求先发生位移；上电已顶在限位时，只要新反馈连续稳定约 100 ms，也可标零。
   注意：如果中途被别的物体挡住，同样会判成“顶死”并在此标零，所以标零时要保证夹爪空载。 */
/* 下面的宏只影响 Arm_Grip_Stall_Zero()。 */
#define GRIP_ZERO_DIR        (+1) /* +1: push in +direction, -1: push in -direction */
#define GRIP_ZERO_TORQUE     (0.55f) /* 纯力矩顶，单位 N·m，只受这个宏控制 */
#define GRIP_ZERO_STILL_EPS  (0.001f) /* 相对本次稳定窗口起点的位置变化小于此值 */
#define GRIP_ZERO_STILL_CNT  (5U)     /* 连续 5 拍没动 = 到死点 */
#define GRIP_ZERO_STEP_MS    (20U)
#define GRIP_ZERO_MAX_STEP   (600U)   /* 20 ms x 600 = 12 s 上限 */

/* Arm_Grip_Stall_Zero() —— 夹爪堵转找零（上电初始化时调用一次）
 * 参数/返回：无
 * 做法：Kp/Kd=0、只给恒定力矩 -> 每 20 ms 采一次反馈 -> 统计“不再移动”的连续拍数
 *       -> 达到 GRIP_ZERO_STILL_CNT 认为顶到死点 -> 发 EEPROM 存零帧(0xFE) -> 撤力矩
 * 失败处理：整个过程最多 12 s；超时、使能丢失、反馈超时、存零后读回校验不过，
 *           都会置 arm_output_diag.startup_failed，由 Arm_L_Init() 转 POWERLESS。
 * 注意：期间 Arm_PrepareCommand() 照常准备快照，恒定力矩通过 torque 字段生效。
 */
static void Arm_Grip_Stall_Zero(void)
{
    float reference = joint_L_motor[6].position;
    float last_position = reference;
    uint32_t last_count = joint_L_motor[6].update_count;
    uint8_t still = 0U, zero_confirmed = 0U;
    uint32_t start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    ArmGripZeroDiagnostics diag = {0};
    diag.result = GRIP_ZERO_RUNNING;
    diag.start_position = diag.last_position = reference;
    arm_grip_zero_diag = diag;
    ArmData.Grip.Kp = ArmData.Grip.Kd = 0.0f; // 关掉位置环：否则会与恒定力矩对抗
    ArmData.Grip.send_set = reference;
    ArmData.Grip.torque = (float)GRIP_ZERO_DIR * GRIP_ZERO_TORQUE; // 恒定力矩朝死点方向顶，正负由 GRIP_ZERO_DIR 决定
    Arm_PrepareAndPublish(); // 首次发布：将新设置的力矩更新到快照中
    for (uint32_t n=0; n<GRIP_ZERO_MAX_STEP; ++n)
    {
        Arm_PrepareAndPublish();  // 每拍重新收集并发布，刷新时间戳防止输出层超时
        osDelay(GRIP_ZERO_STEP_MS); // 20 ms 采样节拍
        joint_motor_measure_t feedback = Arm_ReadGripFeedback();
        diag.last_position = feedback.position;
        diag.feedback_age_ms = xTaskGetTickCount()*portTICK_PERIOD_MS-feedback.update_tick;
        diag.max_travel = fmaxf(diag.max_travel, fabsf(feedback.position-reference));
        if (!Arm_Output_AllEnabled()) 
        {
            diag.result = GRIP_ZERO_ENABLE_LOST;
            break;
        }
        if (diag.feedback_age_ms > 40U)
     {
            diag.result = GRIP_ZERO_FEEDBACK_STALE;
            break;
        }
        if (feedback.update_count == last_count) continue; /* no stale stall samples */ // 没有新帧就跳过，避免用重复样本误判“没动”
        last_count = feedback.update_count;
        /* 从第一拍开始统计稳定的新反馈；无需先移动。位置超出窗口则重新计数。 */
        if (fabsf(feedback.position-last_position)<GRIP_ZERO_STILL_EPS) still++;
        else { still=0U; last_position=feedback.position; }
        if (still>=GRIP_ZERO_STILL_CNT) break;
    }
    ArmData.Grip.torque=0.0f; // 先撤力矩再判断结果，避免长时间顶着死点
    diag.still = still;
    diag.elapsed_ms = xTaskGetTickCount()*portTICK_PERIOD_MS-start_ms;
    arm_grip_zero_diag = diag;
    Arm_PrepareAndPublish();
    osDelay(8);
    if (still>=GRIP_ZERO_STILL_CNT && Arm_Output_AllEnabled()) 
    { // 连续停稳且仍使能，即可写零点
        uint8_t zero_frame[8] = {0xFF, 0xFF, 0xFF, 0xFF,
                                 0xFF, 0xFF, 0xFF, 0xFE}; // 保存当前位置为零点
        uint32_t sent_ms=xTaskGetTickCount()*portTICK_PERIOD_MS;
        if (canx_send_data(&hfdcan1,7U,zero_frame,8U)==HAL_OK) 
        { // 存零帧只发一次，之后靠读回确认
            diag.zero_sent = 1U;
            diag.result = GRIP_ZERO_READBACK_TIMEOUT;
            /* EEPROM command is sent once; require a fresh zero-position reply. */
            for (unsigned n=0;n<10;++n)
         {
                Arm_PublishCurrent();  // 快速循环（10ms），仅发布
                osDelay(10);
                joint_motor_measure_t feedback = Arm_ReadGripFeedback();
                diag.last_position = feedback.position;
                diag.feedback_age_ms = xTaskGetTickCount()*portTICK_PERIOD_MS-feedback.update_tick;
                if (!Arm_Output_AllEnabled()) 
                {
                    diag.result = GRIP_ZERO_VERIFY_ENABLE_LOST;
                    break;
                }
                if (feedback.update_count!=last_count &&
                    (int32_t)(feedback.update_tick-sent_ms)>0 && fabsf(feedback.position)<0.05f) 
                { // 必须是存零之后的新帧且读数接近 0 才认可
                    diag.result = GRIP_ZERO_OK;
                    zero_confirmed=1U;break;
                }
            }
        } 
        else 
        {
            diag.result = GRIP_ZERO_SEND_FAILED;
        }
    } 
    else if (diag.result == GRIP_ZERO_RUNNING)
    {
        diag.result = (still>=GRIP_ZERO_STILL_CNT) ?
            GRIP_ZERO_VERIFY_ENABLE_LOST : GRIP_ZERO_STALL_TIMEOUT;
    }
    diag.elapsed_ms = xTaskGetTickCount()*portTICK_PERIOD_MS-start_ms;
    arm_grip_zero_diag = diag;
    if (!zero_confirmed) arm_output_diag.startup_failed=1U; // 标零失败：置启动失败，Arm_L_Init 会转 POWERLESS
    ArmData.Grip.send_set = Arm_ReadGripFeedback().position; // 无论成败都把目标同步到最新实测值
    Arm_PrepareAndPublish();
}

/* 保存回零现场，供 Watch 定位未到位关节及反馈/使能故障。 */
static void Arm_HomeRecord(const joint_motor_measure_t fb[7], uint32_t now, uint32_t start)
{
    const float targets[7] = {J1_zero_position, J2_zero_position, J3_zero_position,
                             J4_zero_position, J5_zero_position, J6_zero_position,
                             grip_zero_position};
    arm_home_diag.elapsed_ms = now - start;
    for (uint8_t i = 0U; i < 7U; ++i) 
    {
        arm_home_diag.error[i] = fb[i].position - targets[i];
        arm_home_diag.velocity[i] = fb[i].velocity;
        arm_home_diag.feedback_age_ms[i] = now - fb[i].update_tick;
        arm_home_diag.state[i] = arm_output_diag.state[i];
        arm_home_diag.status[i] = fb[i].status;
    }
}

/* Arm_L_Init() —— 机械臂上电初始化（只在任务启动时调用一次）
 * 流程：先空发布几次 -> 等 950 ms 让电机上电稳定 -> 申请输出层使能
 *       -> 等全部电机反馈（最多 1.5 s，超时置 startup_failed 并转 POWERLESS）
 *       -> 用实测位置填 position / send_set / Initial_Total_position
 *       -> Arm_Grip_Stall_Zero() 夹爪堵转标零
 *       -> 斜坡状态量对齐实测位置，再写入各关节 PD 增益
 *       -> do-while 以 0.0038/0.0035 rad 每拍把各关节收回 zero_position，
 *          每拍发布一次并 osDelay(5)，最多 20 s；超时同样转 POWERLESS
 *       -> 清空累加器、斜坡状态量重置到零点、置 Arm_L_Ready=1
 * 注意：全程阻塞调用者（可能十几秒），此时主循环尚未开始。
 */
void Arm_L_Init(void)
{
    /* ================= 阶段 1：上电稳定期 ================= */
    Arm_PrepareAndPublish();
    osDelay(950); /* 电机驱动器上电稳定窗口 */
    button_init(&left_button_btn);
    button_init(&right_button_btn);

    /* ================= 阶段 2：请求使能并等待反馈 ================= */
    Arm_PrepareAndPublish();
    Arm_Output_Start();  // 请求输出层使能
    uint32_t start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    while (!Arm_Output_AllEnabled() &&
           xTaskGetTickCount() * portTICK_PERIOD_MS - start_ms < 1500U)
	{
        Arm_PrepareAndPublish();
        osDelay(8);
    }
    if (!Arm_Output_AllEnabled()) 
	{
        arm_output_diag.startup_failed = 1U;
        ArmData.ArmMode = ARM_POWERLESS;
        Arm_Powerless();
        Arm_PrepareAndPublish();
        return;  /* 使能失败，需要复位重试 */
    }

    /* ================= 阶段 3：保存零点并收集反馈 ================= */
//	Arm_SaveMotorZero();

    /* 等待 7 个电机反馈到达（最多 1.5s），超时则失败返回 */
    if (!Arm_WaitMotorFeedback(1500U)) {
        arm_output_diag.startup_failed = 1U;
        ArmData.ArmMode = ARM_POWERLESS;
        Arm_Powerless();
        Arm_PrepareAndPublish();
        return;  /* 反馈超时 */
    }

    /* 从 CAN 反馈收集当前位置 */
    Arm_CollectMotorPositions();

    /* 初始化目标位置为当前实测位置（防止上电阶跃） */
    Arm_InitTargetPositions();
	
//	RC_contorl.RC = get_RC_measure_point();  //VT13_rc_ctrl
	


    /* ================= 阶段 4：夹爪堵转标零 ================= */
    /* 夹爪：堵转找机械死点 -> 存零点 -> 撤力矩 */
    Arm_Grip_Stall_Zero();
    if (arm_output_diag.startup_failed || !Arm_Output_AllEnabled()) {
        arm_output_diag.startup_failed = 1U;
        Arm_Powerless();
        Arm_PrepareAndPublish();
        return;  /* 夹爪标零失败 */
    }

    /* 夹爪标零完成，收集最新反馈并初始化 */
    Arm_CollectMotorPositions();
    ArmData.Grip.position = Arm_ReadGripFeedback().position;
    ArmData.Grip.send_set = ArmData.Grip.position;
    ArmData.Grip.Initial_Total_position = ArmData.Grip.position;

    /* ================= 阶段 5：初始化斜坡和增益，执行回零 ================= */
    Arm_InitPositionRamps();
    Arm_InitJointGains();

    uint32_t home_start = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t settle_start = 0U;     // 稳定计时起点
    uint8_t all_settled = 0U;       // 是否满足到位条件
    joint_motor_measure_t fb_snapshot[7]; // 反馈快照
    uint32_t last_update_counts[7] = {0};  // 上次的 update_count，用于检测新反馈
    ArmHomeDiagnostics home_diag = {0};
    home_diag.result = 1U;
    arm_home_diag = home_diag;

    do {
        uint8_t home_enabled = Arm_Output_AllEnabled();

        /* 在临界区内复制七轴反馈，避免读到半更新状态 */
        taskENTER_CRITICAL();
        for (uint8_t i = 0; i < 7; ++i) {
            fb_snapshot[i] = joint_L_motor[i];
        }
        taskEXIT_CRITICAL();

        /* 反馈新鲜度检查：任何一轴超过 100 ms 未更新就重置稳定计时 */
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        Arm_HomeRecord(fb_snapshot, now, home_start);
        /* 分别锁存失能和超时原因，保留撤力前的反馈。 */
        if (!home_enabled || now - home_start > 20000U) {
            arm_home_diag.result = !home_enabled ? 3U : 4U;
            arm_output_diag.startup_failed = 1U;
            Arm_Powerless();
            Arm_PrepareAndPublish();
            return;
        }
        uint8_t feedback_fresh = 1U;
        for (uint8_t i = 0; i < 7; ++i) {
            if (now - fb_snapshot[i].update_tick > 100U) {
                feedback_fresh = 0U;
                all_settled = 0U;
                settle_start = 0U;
                break;
            }
        }

        /* 每拍朝零点走固定步长 */
        arm_ramp_calc(&J1_L_position_set, 0.0038, J1_zero_position);
        arm_ramp_calc(&J2_L_position_set, 0.0035, J2_zero_position);
        arm_ramp_calc(&J3_L_position_set, 0.0035, J3_zero_position);
        arm_ramp_calc(&J4_L_position_set, 0.0035, J4_zero_position);
        arm_ramp_calc(&J5_L_position_set, 0.0035, J5_zero_position);
        arm_ramp_calc(&J6_L_position_set, 0.0035, J6_zero_position);
        arm_ramp_calc(&grip_L_position_set, 0.0035, grip_zero_position);

        /* 把斜坡输出写回目标位置 */
        ArmData.J1.send_set = J1_L_position_set.out;
        ArmData.J2.send_set = J2_L_position_set.out;
        ArmData.J3.send_set = J3_L_position_set.out;
        ArmData.J4.send_set = J4_L_position_set.out;
        ArmData.J5.send_set = J5_L_position_set.out;
        ArmData.J6.send_set = J6_L_position_set.out;
        ArmData.Grip.send_set = grip_L_position_set.out;

        /* 硬限位检查 */
        limitPosition(&ArmData.J1.send_set, J1_Max_position, J1_Min_position);
        limitPosition(&ArmData.J2.send_set, J2_Max_position, J2_Min_position);
        limitPosition(&ArmData.J3.send_set, J3_Max_position, J3_Min_position);
        limitPosition(&ArmData.J4.send_set, J4_Max_position, J4_Min_position);
        limitPosition(&ArmData.J5.send_set, J5_Max_position, J5_Min_position);
        limitPosition(&ArmData.J6.send_set, J6_Max_position, J6_Min_position);
        limitPosition(&ArmData.Grip.send_set, Grip_Max_position, Grip_Min_position);

        /* 发布本拍目标，刷新时间戳保持命令有效 */
        Arm_PrepareAndPublish();

        /* 检查斜坡是否全部计算完成（输出已到达目标） */
        uint8_t ramps_done = (J1_L_position_set.out == J1_zero_position &&
                              J2_L_position_set.out == J2_zero_position &&
                              J3_L_position_set.out == J3_zero_position &&
                              J4_L_position_set.out == J4_zero_position &&
                              J5_L_position_set.out == J5_zero_position &&
                              J6_L_position_set.out == J6_zero_position &&
                              grip_L_position_set.out == grip_zero_position);
        arm_home_diag.ramps_done = ramps_done;

        /* 斜坡完成后，检查实际位置是否到位且稳定 */
        if (ramps_done) {
            float zero_targets[7] = {J1_zero_position, J2_zero_position, J3_zero_position,
                                     J4_zero_position, J5_zero_position, J6_zero_position,
                                     grip_zero_position};
            uint8_t in_window = feedback_fresh;
            uint8_t has_new_feedback = 1;

            /* 检查所有轴：位置误差、速度、是否有新反馈 */
            for (uint8_t i = 0; i < 7; ++i) {
                float err = fabsf(fb_snapshot[i].position - zero_targets[i]);
                float vel = fabsf(fb_snapshot[i].velocity);
                if (err >= 0.05f || vel >= 0.1f) {
                    in_window = 0;
                    break;
                }
                /* 如果 update_count 没变化，说明没有新反馈参与稳定确认 */
                if (fb_snapshot[i].update_count == last_update_counts[i]) {
                    has_new_feedback = 0;
                }
            }

            arm_home_diag.in_window = in_window;
            arm_home_diag.has_new_feedback = has_new_feedback;
            if (in_window && has_new_feedback) {
                if (!all_settled) {
                    /* 首次进入窗口，开始计时 */
                    all_settled = 1U;
                    settle_start = xTaskGetTickCount() * portTICK_PERIOD_MS;
                } else {
                    /* 已在窗口内，检查是否已稳定 100 ms */
                    if (xTaskGetTickCount() * portTICK_PERIOD_MS - settle_start >= 100U) {
                        J1_J2_J3_J4_J5_J6_J7_init_flag = 0; // 退出循环
                    }
                }
                /* 更新 last_update_counts */
                for (uint8_t i = 0; i < 7; ++i) {
                    last_update_counts[i] = fb_snapshot[i].update_count;
                }
            } else if (!in_window) {
                /* 越出窗口或反馈过期才重置；尚无新帧时等待，保留稳定起点。
                 * 完成确认仍须七轴均有新反馈，不能仅凭旧样本到时放行。 */
                all_settled = 0U;
                settle_start = 0U;
            }
        }

        arm_home_diag.settle_ms = all_settled ?
            xTaskGetTickCount()*portTICK_PERIOD_MS - settle_start : 0U;
        osDelay(5);
    } while (J1_J2_J3_J4_J5_J6_J7_init_flag);

    /* ================= 阶段 6：初始化完成，清空累加器 ================= */
    Arm_ClearControlOffsets();
    Arm_ResetPositionRamps();
    Arm_PrepareAndPublish();
    vTaskDelay(10);  /* 让最后一帧发出去 */

    /* 重新读取反馈并确认到位状态 */
    taskENTER_CRITICAL();
    for (uint8_t i = 0; i < 7; ++i) {
        fb_snapshot[i] = joint_L_motor[i];
    }
    taskEXIT_CRITICAL();

    uint32_t final_check_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    Arm_HomeRecord(fb_snapshot, final_check_time, home_start);
    /* 置 READY 前最后一次健康检查 */
    if (!Arm_Output_AllEnabled()) {
        arm_home_diag.result = 5U;
        arm_output_diag.startup_failed = 1U;
        Arm_Powerless();
        Arm_PrepareAndPublish();
        return;
    }
    float zero_targets[7] = {J1_zero_position, J2_zero_position, J3_zero_position,
                             J4_zero_position, J5_zero_position, J6_zero_position,
                             grip_zero_position};

    for (uint8_t i = 0; i < 7; ++i) {
        /* 反馈新鲜度检查 */
        if (final_check_time - fb_snapshot[i].update_tick > 100U) {
            arm_home_diag.result = 6U;
            arm_output_diag.startup_failed = 1U;
            Arm_Powerless();
            Arm_PrepareAndPublish();
            return;
        }
        /* 位置和速度最终确认 */
        float err = fabsf(fb_snapshot[i].position - zero_targets[i]);
        float vel = fabsf(fb_snapshot[i].velocity);
        if (err >= 0.05f || vel >= 0.1f) {
            arm_home_diag.result = 7U;
            arm_output_diag.startup_failed = 1U;
            Arm_Powerless();
            Arm_PrepareAndPublish();
            return;
        }
    }

    arm_home_diag.result = 2U;
    Arm_L_Ready = 1;  /* 就绪标志 */
}

/* data_updata() —— 每拍刷新输入状态
 * 参数/返回：无
 * 调用：Arm_L_Task() 主循环每拍的第一件事。
 * 说明：先从 CAN 反馈同步实测位置，再调用 GetArmMode()。GetArmMode() 统一负责
 *       拨杆解析、消抖、安全覆盖和模式切换动作，避免主循环重复维护 prev_ArmMode。
 */
void data_updata(void)
{
	/* 先取同一时刻的 7 轴反馈，再由 GetArmMode() 完成模式解析和切换动作。 */
	Arm_UpdateMeasuredPositions();
	ArmData.ArmMode = GetArmMode();
}

/* GetArmMode() —— 由 VT13 遥控器两个拨杆的组合解析并切换工作模式
 * 参数/返回：返回消抖后的稳定模式（见 Arm_mode_t）
 * 调用：data_updata() 每拍调用；该函数同时执行一次性的模式进入动作
 * 消抖：模式与夹爪力矩开关各自要连续 3 拍一致才生效；遥控失联（任一拨杆数据为 0）
 *       立即生效，不消抖。
 */
uint8_t GetArmMode(void)
{
    uint8_t mode = 0;
    uint8_t right, left;
    taskENTER_CRITICAL(); // 拨杆值在临界区内取，避免与中断更新撕裂
    right=(uint8_t)RCData.rc.s[RC_Right]; left=(uint8_t)RCData.rc.s[RC_Left];
    taskEXIT_CRITICAL();

    uint8_t switches_valid = right >= 1U && right <= 3U && left >= 1U && left <= 3U;
    static uint8_t retry_candidate, retry_samples;
    if (switches_valid) {
        uint8_t selected = switch_is_down(right) && switch_is_down(left);
        if (selected != retry_candidate) { retry_candidate = selected; retry_samples = 1U; }
        else if (retry_samples < 3U) retry_samples++;
        if (retry_samples >= 3U) retry_powerless_selected = retry_candidate;
    } else {
        /* Retain the last valid selection, so reconnecting in the same
         * position cannot silently authorize another retry. */
        retry_samples = 0U;
    }

    /* 左上右上：首次接受 PC 目标前无力；接管后丢失目标时保持位置。 */
	if (switch_is_up(right) && switch_is_up(left))
	{
		// mode = ARM_RC;	
        mode = ARM_MODE_PC_CONTROL;
	}
    else if (switch_is_down(right)&&switch_is_up(left)) 
	{
		mode = ARM_RC;
	}

	else if (switch_is_mid(right)&&switch_is_up(left))  
	{
			mode = TAKE_POSE_2;
	}
	else if (switch_is_mid(right)&&switch_is_mid(left))  
	{
            mode = TAKE_POSE_1;
	}
	else if (switch_is_mid(right)&&switch_is_down(left))  
	{
            mode = ARM_RESET;
	}
	// 	else if (switch_is_down(right)&&switch_is_up(left))  
	// {
	// 		mode = ARM_RC;
	// }

	
//	else if (left_button_press_count % 2 == 0&&right_button_press_count % 2 == 0)  //left_button_press_count % 2 == 0&&right_button_press_count % 2 == 0
//	{
//			mode = CHASSIS_MOVEMENT_ARM_KEEP;
//	}
//	else if (RC_contorl.RC->rc.shutter == 1)  //RC_contorl.RC->rc.shutter == 1
//	{
//			mode = ARM_POWERLESS;
//	}
	else if (switch_is_down(right)&&switch_is_down(left))  //RC_contorl.RC->rc.shutter == 1
	{
			mode = ARM_POWERLESS;
	}
	
	else
	{
			mode = ARM_POWERLESS;
	}
    static uint8_t candidate = ARM_POWERLESS, stable = ARM_POWERLESS, samples; // 3 拍一致消抖：候选模式连续 3 次相同才更新 stable
    static uint8_t grip_candidate, grip_samples;
    static uint8_t previous_mode = ARM_POWERLESS;
    uint8_t grip = switch_is_up(right) && switch_is_mid(left); // 夹爪力矩模式开关：右上 + 左中 组合
    if (mode != candidate) { candidate = mode; samples = 1U; }
    else if (samples < 3U) samples++;
    if (samples >= 3U) stable = candidate;
    if (grip != grip_candidate) { grip_candidate = grip; grip_samples = 1U; }
    else if (grip_samples < 3U) grip_samples++;
    if (grip_samples >= 3U) grip_torque_mode = grip_candidate; // 同样 3 拍一致才写 grip_torque_mode（影响 Arm_Publish 的夹爪过渡）
    /* Lost remote is immediately powerless, without debounce delay. */
    if (!switches_valid) { // 遥控失联/非法拨杆值：立即无力，重新消抖
        stable = ARM_POWERLESS; grip_torque_mode = 0U;
        candidate = ARM_POWERLESS; samples = 0U;
        grip_candidate = 0U; grip_samples = 0U;
    }

    /* 安全覆盖必须在模式切换动作之前执行，避免故障时进入受控模式。 */
    if (!Arm_L_Ready || arm_output_diag.bus_inhibit) {
        stable = ARM_POWERLESS;
        grip_torque_mode = 0U;
    }

    /* 模式解析、消抖、故障覆盖和进入动作集中在此处。 */
    if (stable != previous_mode) 
    {
        if (stable == ARM_RC) 
        {
            /* 统一使用 Initial_Total_position 作基准，与限位逻辑对齐。 */
            ArmData.J1.RC_Sum_Value = ArmData.J1.position - ArmData.J1.Initial_Total_position;
            ArmData.J2.RC_Sum_Value = ArmData.J2.position - ArmData.J2.Initial_Total_position;
            ArmData.J3.RC_Sum_Value = ArmData.J3.position - ArmData.J3.Initial_Total_position;
            ArmData.J4.RC_Sum_Value = ArmData.J4.position - ArmData.J4.Initial_Total_position;
            ArmData.J5.RC_Sum_Value = ArmData.J5.position - ArmData.J5.Initial_Total_position - Custom_RC[4].DM_CC_receive - ArmData.J5.Key_Control_Value;
            ArmData.J6.RC_Sum_Value = ArmData.J6.position - ArmData.J6.Initial_Total_position - Custom_RC[5].DM_CC_receive - ArmData.J6.Key_Control_Value;
            ArmData.Grip.RC_Sum_Value = ArmData.Grip.position - ArmData.Grip.Initial_Total_position;
        }

        if (stable == TAKE_POSE_1 || stable == TAKE_POSE_2 || stable == ARM_RESET )
        {
            /* 进入预设位姿时，从当前实测位置开始斜坡。 */
            for (uint8_t i = 0U; i < 7U; ++i) {
                arm_data_ramp_init(arm_joint_ramp[i], 1.0f,
                                   arm_joint_control[i]->position);
            }
        }

        if (stable == ARM_MODE_PC_CONTROL) 
        {
            Arm_PC_Control_Enter();
        } 
        else 
        {
            Arm_USB_PC_Disarm();
        }

        previous_mode = stable;
    }

    return stable;
}

/* ==================== 预设位姿模式函数 ====================
 * TAKE_Grip_1 / ARM_RESET_1 / PASS_WATER_1 / TAKE_WATER_1 结构完全一致：
 * 写 PD -> 取宏定义的预设目标 -> 设各轴斜坡步长 -> 硬限位 -> 斜坡
 * -> 写 ArmData.*.send_set。它们每拍被调用一次、目标固定，所以机械臂会以
 * 固定步长平滑收敛到预设位姿；本身不带状态机，也不阻塞。
 */
/*************************** 模式 ****************************/


/* TAKE_POSE_1() —— 预设位姿
 * 参数/返回：无
 * 调用：右拨杆中、左拨杆下，主循环 TAKE_POSE 分支每拍调用一次。
 * 目标：J*_POSE_position 和 Grip_POSE_position_1，经限位和斜坡后发布。
 * CAN 帧由 Moter_Task 发送；输出层故障时仍会禁止发送或撤掉增益。
 */
void TAKE_pose_1(void)
{
	//static int take_water = 0;
	ArmData.J1.Kp = J1_PID_KP;ArmData.J1.Kd = J1_PID_KD;
	ArmData.J2.Kp = J2_PID_KP;ArmData.J2.Kd = J2_PID_KD;
	ArmData.J3.Kp = J3_PID_KP;ArmData.J3.Kd = J3_PID_KD;
	ArmData.J4.Kp = J4_PID_KP;ArmData.J4.Kd = J4_PID_KD;
	ArmData.J5.Kp = J5_PID_KP;ArmData.J5.Kd = J5_PID_KD;
	ArmData.J6.Kp = J6_PID_KP;ArmData.J6.Kd = J6_PID_KD;
	ArmData.Grip.Kp = Grip_PID_KP;ArmData.Grip.Kd = Grip_PID_KD;

	J1_send_set = J1_POSE_position;
	J2_send_set = J2_POSE_position;
	J3_send_set = J3_POSE_position;
	J4_send_set = J4_POSE_position;
	J5_send_set = J5_POSE_position;
	J6_send_set = J6_POSE_position;
	Grip_send_set = Grip_POSE_position_1;

//	Grip_send_set = grip_zero_position;

	J1_ramp= 0.0090,J2_ramp= 0.002,J3_ramp= 0.0035,J4_ramp= 0.0032,J5_ramp= 0.002,J6_ramp= 0.015,Grip_ramp= 0.030;  //锟斤拷锟斤拷锟斤拷


	limitPosition(&J1_send_set,J1_Max_position,J1_Min_position);
	limitPosition(&J2_send_set,J2_Max_position,J2_Min_position);
	limitPosition(&J3_send_set,J3_Max_position,J3_Min_position);
	limitPosition(&J4_send_set,J4_Max_position,J4_Min_position);
	limitPosition(&J5_send_set,J5_Max_position,J5_Min_position);
	limitPosition(&J6_send_set,J6_Max_position,J6_Min_position);
	limitPosition(&Grip_send_set,Grip_Max_position,Grip_Min_position);

	arm_ramp_calc(&J1_L_position_set,J1_ramp,J1_send_set);
	arm_ramp_calc(&J2_L_position_set,J2_ramp,J2_send_set);
	arm_ramp_calc(&J3_L_position_set,J3_ramp,J3_send_set);
	arm_ramp_calc(&J4_L_position_set,J4_ramp,J4_send_set);
	arm_ramp_calc(&J5_L_position_set,J5_ramp,J5_send_set);
	arm_ramp_calc(&J6_L_position_set,J6_ramp,J6_send_set);
	arm_ramp_calc(&grip_L_position_set,Grip_ramp,Grip_send_set);

	/*���λ�ø�ֵ*/
	ArmData.J1.send_set = J1_L_position_set.out;
	ArmData.J2.send_set = J2_L_position_set.out;
	ArmData.J3.send_set = J3_L_position_set.out;
	ArmData.J4.send_set = J4_L_position_set.out;
	ArmData.J5.send_set = J5_L_position_set.out;
	ArmData.J6.send_set = J6_L_position_set.out;
	ArmData.Grip.send_set = grip_L_position_set.out;
}

void TAKE_pose_2(void)
{
	//static int take_water = 0;
	ArmData.J1.Kp = J1_PID_KP;ArmData.J1.Kd = J1_PID_KD;
	ArmData.J2.Kp = J2_PID_KP;ArmData.J2.Kd = J2_PID_KD;
	ArmData.J3.Kp = J3_PID_KP;ArmData.J3.Kd = J3_PID_KD;
	ArmData.J4.Kp = J4_PID_KP;ArmData.J4.Kd = J4_PID_KD;
	ArmData.J5.Kp = J5_PID_KP;ArmData.J5.Kd = J5_PID_KD;
	ArmData.J6.Kp = J6_PID_KP;ArmData.J6.Kd = J6_PID_KD;
	ArmData.Grip.Kp = Grip_PID_KP;ArmData.Grip.Kd = Grip_PID_KD;

	J1_send_set = J1_POSE_position_2;
	J2_send_set = J2_POSE_position_2;
	J3_send_set = J3_POSE_position_2;
	J4_send_set = J4_POSE_position_2;
	J5_send_set = J5_POSE_position_2;
	J6_send_set = J6_POSE_position_2;
	Grip_send_set = Grip_POSE_position_2;

//	Grip_send_set = grip_zero_position;

	J1_ramp= 0.0090,J2_ramp= 0.002,J3_ramp= 0.0035,J4_ramp= 0.0032,J5_ramp= 0.002,J6_ramp= 0.015,Grip_ramp= 0.030;  //锟斤拷锟斤拷锟斤拷


	limitPosition(&J1_send_set,J1_Max_position,J1_Min_position);
	limitPosition(&J2_send_set,J2_Max_position,J2_Min_position);
	limitPosition(&J3_send_set,J3_Max_position,J3_Min_position);
	limitPosition(&J4_send_set,J4_Max_position,J4_Min_position);
	limitPosition(&J5_send_set,J5_Max_position,J5_Min_position);
	limitPosition(&J6_send_set,J6_Max_position,J6_Min_position);
	limitPosition(&Grip_send_set,Grip_Max_position,Grip_Min_position);

	arm_ramp_calc(&J1_L_position_set,J1_ramp,J1_send_set);
	arm_ramp_calc(&J2_L_position_set,J2_ramp,J2_send_set);
	arm_ramp_calc(&J3_L_position_set,J3_ramp,J3_send_set);
	arm_ramp_calc(&J4_L_position_set,J4_ramp,J4_send_set);
	arm_ramp_calc(&J5_L_position_set,J5_ramp,J5_send_set);
	arm_ramp_calc(&J6_L_position_set,J6_ramp,J6_send_set);
	arm_ramp_calc(&grip_L_position_set,Grip_ramp,Grip_send_set);

	/*���λ�ø�ֵ*/
	ArmData.J1.send_set = J1_L_position_set.out;
	ArmData.J2.send_set = J2_L_position_set.out;
	ArmData.J3.send_set = J3_L_position_set.out;
	ArmData.J4.send_set = J4_L_position_set.out;
	ArmData.J5.send_set = J5_L_position_set.out;
	ArmData.J6.send_set = J6_L_position_set.out;
	ArmData.Grip.send_set = grip_L_position_set.out;
}

void ARM_reset(void)
{
	//static int take_water = 0;
	ArmData.J1.Kp = J1_PID_KP;ArmData.J1.Kd = J1_PID_KD;
	ArmData.J2.Kp = J2_PID_KP;ArmData.J2.Kd = J2_PID_KD;
	ArmData.J3.Kp = J3_PID_KP;ArmData.J3.Kd = J3_PID_KD;
	ArmData.J4.Kp = J4_PID_KP;ArmData.J4.Kd = J4_PID_KD;
	ArmData.J5.Kp = J5_PID_KP;ArmData.J5.Kd = J5_PID_KD;
	ArmData.J6.Kp = J6_PID_KP;ArmData.J6.Kd = J6_PID_KD;
	ArmData.Grip.Kp = Grip_PID_KP;ArmData.Grip.Kd = Grip_PID_KD;

	J1_send_set = J1_reset_position;
	J2_send_set = J2_reset_position;
	J3_send_set = J3_reset_position;
	J4_send_set = J4_reset_position;
	J5_send_set = J5_reset_position;
	J6_send_set = J6_reset_position;
	Grip_send_set = Grip_reset_position;

//	Grip_send_set = grip_zero_position;

	J1_ramp= 0.0090,J2_ramp= 0.002,J3_ramp= 0.0035,J4_ramp= 0.0032,J5_ramp= 0.002,J6_ramp= 0.015,Grip_ramp= 0.030;  


	limitPosition(&J1_send_set,J1_Max_position,J1_Min_position);
	limitPosition(&J2_send_set,J2_Max_position,J2_Min_position);
	limitPosition(&J3_send_set,J3_Max_position,J3_Min_position);
	limitPosition(&J4_send_set,J4_Max_position,J4_Min_position);
	limitPosition(&J5_send_set,J5_Max_position,J5_Min_position);
	limitPosition(&J6_send_set,J6_Max_position,J6_Min_position);
	limitPosition(&Grip_send_set,Grip_Max_position,Grip_Min_position);

	arm_ramp_calc(&J1_L_position_set,J1_ramp,J1_send_set);
	arm_ramp_calc(&J2_L_position_set,J2_ramp,J2_send_set);
	arm_ramp_calc(&J3_L_position_set,J3_ramp,J3_send_set);
	arm_ramp_calc(&J4_L_position_set,J4_ramp,J4_send_set);
	arm_ramp_calc(&J5_L_position_set,J5_ramp,J5_send_set);
	arm_ramp_calc(&J6_L_position_set,J6_ramp,J6_send_set);
	arm_ramp_calc(&grip_L_position_set,Grip_ramp,Grip_send_set);

	ArmData.J1.send_set = J1_L_position_set.out;
	ArmData.J2.send_set = J2_L_position_set.out;
	ArmData.J3.send_set = J3_L_position_set.out;
	ArmData.J4.send_set = J4_L_position_set.out;
	ArmData.J5.send_set = J5_L_position_set.out;
	ArmData.J6.send_set = J6_L_position_set.out;
	ArmData.Grip.send_set = grip_L_position_set.out;
}


/* Arm_Powerless() —— 无力模式：撤掉所有出力，只保持通信
 * 做法：Kp/Kd 全部写 Joint_Atony_KP_KD(=0)；目标位置取当前实测位置；
 *       位置基准、斜坡状态、遥控累加器一并归零，保证切回其他模式时
 *       不会因为残留目标而突然甩臂。
 * 调用：Arm_L_Task() 在 ArmData.ArmMode == ARM_POWERLESS 时每拍调用。
 */
void Arm_Powerless(void)
{
    ArmData.Grip.torque = 0.0f;
	ArmData.J1.Kp = ArmData.J2.Kp = ArmData.J3.Kp = ArmData.J4.Kp = ArmData.J5.Kp = ArmData.J6.Kp = ArmData.Grip.Kp = ArmData.J1.Kd = ArmData.J2.Kd = ArmData.J3.Kd = ArmData.J4.Kd = ArmData.J5.Kd = ArmData.J6.Kd = ArmData.Grip.Kd = Joint_Atony_KP_KD; // Kp/Kd 全置 0：MIT 模式下等效“不出力”

	ArmData.J1.send_set=ArmData.J1.position;
	ArmData.J2.send_set=ArmData.J2.position;
	ArmData.J3.send_set=ArmData.J3.position;	
	ArmData.J4.send_set=ArmData.J4.position;
	ArmData.J5.send_set=ArmData.J5.position;
	ArmData.J6.send_set=ArmData.J6.position;
	ArmData.Grip.send_set=ArmData.Grip.position;
/* 位置基准重置为实测值：后续增量控制都相对这里累加。 */

	ArmData.J1.Initial_Total_position=ArmData.J1.position;
	ArmData.J2.Initial_Total_position=ArmData.J2.position;
	ArmData.J3.Initial_Total_position=ArmData.J3.position;
	ArmData.J4.Initial_Total_position=ArmData.J4.position;
	ArmData.J5.Initial_Total_position=ArmData.J5.position;
	ArmData.J6.Initial_Total_position=ArmData.J6.position;
	ArmData.Grip.Initial_Total_position=ArmData.Grip.position;
	
/* 斜坡状态 out 同步到实测位置：切回遥控模式时不会从旧目标起跳。 */

	J1_L_position_set.out = ArmData.J1.position;
	J2_L_position_set.out = ArmData.J2.position;
	J3_L_position_set.out = ArmData.J3.position;
	J4_L_position_set.out = ArmData.J4.position;
	J5_L_position_set.out = ArmData.J5.position;
	J6_L_position_set.out = ArmData.J6.position;
	grip_L_position_set.out = ArmData.Grip.position;
	
/* 遥控/按键累加器清零：下次进入遥控模式从当前位置重新开始累加。 */
	ArmData.J1.RC_Sum_Value = 0;
	ArmData.J2.RC_Sum_Value = 0;
	ArmData.J3.RC_Sum_Value = 0;
	ArmData.J4.RC_Sum_Value = 0;
	ArmData.J5.RC_Sum_Value = 0;
	ArmData.J6.RC_Sum_Value = 0;
	ArmData.Grip.RC_Sum_Value = 0;

}

/* PC 控制链路的持久状态（static，仅本文件可见）
 *   pc_motion[7]     每关节一维运动插补器（实现见 arm_motion.h）
 *   pc_last_ms       上一拍时间戳(ms)，用于计算 dt
 *   pc_phase         首次接管前无力，跟随过有效目标后才允许保持
 */
static ArmMotion pc_motion[7];
static uint32_t pc_last_ms;
static enum { PC_READ_ONLY, PC_TRACKING, PC_HOLDING } pc_phase = PC_READ_ONLY;

/* Arm_PC_Control_Enter() —— 进入 PC 模式时的一次性复位
 * 参数/返回：无
 * 调用：模式切换检测中，切入 ARM_MODE_PC_CONTROL 的那一拍调用一次。
 * 作用：回到只读无力状态，强制下一帧有效目标到达时用实测位置
 *       重建插补器，避免沿用进入前的旧目标。
 */
void Arm_PC_Control_Enter(void)
{
    pc_phase = PC_READ_ONLY;
    pc_last_ms = xTaskGetTickCount()*portTICK_PERIOD_MS; // 记录时间基准：下一拍的 dt 从这里起算
}

/* Arm_PC_Control() —— 上位机（ROS 2 / MoveIt）控制
 * 输入：Arm_USB_Get_PC_Command() 返回目标位置 pc_target[7] 与本拍速度上限 vmax[7]
 * 流程：取目标 -> 只读时撤力，否则写 PD 增益 -> 插补或保持 -> 静态限位
 *       -> 同步斜坡状态量
 * 注意：目标有效时不做 arm_ramp 斜坡（插补已在 Arm_Motion_Step 里做）；丢包/超时/
 *       断开时锁存进入失效时的位置并保持（安全动作），不会掉臂。
 */
void Arm_PC_Control(void)
{
	static float pc_target[7]; // 本拍 PC 目标位置（下标 0..6 ↔ 电机 ID 1..7）
	static float hold_position[7]; // 首次失效时锁存的保持位置
	uint8_t got_target;
    float vmax[7];

    got_target = Arm_USB_Get_PC_Command(pc_target, vmax);
    uint32_t now_ms = xTaskGetTickCount()*portTICK_PERIOD_MS;
    float dt = (float)(now_ms-pc_last_ms)*0.001f;
    pc_last_ms = now_ms;
    for (uint8_t joint = 0; joint < 7U; ++joint)
    {
        arm_joint_control[joint]->torque = 0.0f;
    }
    if (!got_target && pc_phase == PC_READ_ONLY)
    {
        Arm_Powerless();
        return;
    }

	/* 璁剧疆姝ｅ父 PID 澧炵泭锛堝弬鏁颁笉鍙橈級 */
	ArmData.J1.Kp = J1_PID_KP; ArmData.J1.Kd = J1_PID_KD; // 每拍重写 PD 增益：保证从其他模式切回后增益正确
	ArmData.J2.Kp = J2_PID_KP; ArmData.J2.Kd = J2_PID_KD;
	ArmData.J3.Kp = J3_PID_KP; ArmData.J3.Kd = J3_PID_KD;
	ArmData.J4.Kp = J4_PID_KP; ArmData.J4.Kd = J4_PID_KD;
	ArmData.J5.Kp = J5_PID_KP; ArmData.J5.Kd = J5_PID_KD;
	ArmData.J6.Kp = J6_PID_KP; ArmData.J6.Kd = J6_PID_KD;
	ArmData.Grip.Kp = J6_PID_KP; ArmData.Grip.Kd = J6_PID_KD; // 夹爪这里沿用 J6 的 PD 宏（Grip 自己的宏在本函数未使用）

    if (got_target) 
    { // 只有拿到有效目标才推进插补器
        const DM_contorl_t *joint[7] = {&ArmData.J1,&ArmData.J2,&ArmData.J3,
            &ArmData.J4,&ArmData.J5,&ArmData.J6,&ArmData.Grip};
        for (unsigned n=0; n<7; ++n) 
        {
            if (pc_phase != PC_TRACKING)
            {
                 pc_motion[n].position=joint[n]->position; pc_motion[n].velocity=0.0f; 
            } // 刚进 PC 模式或上次无目标：用实测位置重置插补器，速度清零
            pc_target[n] = Arm_Motion_Step(&pc_motion[n], pc_target[n], vmax[n], dt); // 逐关节插补：速度上限 vmax[n]、加速度上限 ARM_PC_AMAX_RAD_S2
        }
		/* 目标有效：跟随 PC 指令 */
		pc_phase = PC_TRACKING;
		ArmData.J1.send_set = pc_target[0]; // 直接赋 send_set：插补已在上面的 Arm_Motion_Step 完成，这里不再走 arm_ramp
		ArmData.J2.send_set = pc_target[1];
		ArmData.J3.send_set = pc_target[2];
		ArmData.J4.send_set = pc_target[3];
		ArmData.J5.send_set = pc_target[4];
		ArmData.J6.send_set = pc_target[5];
		ArmData.Grip.send_set = pc_target[6];
	}
	else
	{
		/* 目标无效（超时/断开/CAN 丢失）：锁存并保持当前位置（安全动作 A） */
		if (pc_phase != PC_HOLDING)
		{
			hold_position[0] = ArmData.J1.position; // 只在失效的第一拍锁存一次，之后一直沿用这组位置
			hold_position[1] = ArmData.J2.position;
			hold_position[2] = ArmData.J3.position;
			hold_position[3] = ArmData.J4.position;
			hold_position[4] = ArmData.J5.position;
			hold_position[5] = ArmData.J6.position;
			hold_position[6] = ArmData.Grip.position;
			pc_phase = PC_HOLDING;
		}
		ArmData.J1.send_set = hold_position[0]; // 保持位置不变：既不跟随 PC，也不继续插补
		ArmData.J2.send_set = hold_position[1];
		ArmData.J3.send_set = hold_position[2];
		ArmData.J4.send_set = hold_position[3];
		ArmData.J5.send_set = hold_position[4];
		ArmData.J6.send_set = hold_position[5];
		ArmData.Grip.send_set = hold_position[6];
	}

	/* 标准静态限位（防御深度）。PC 模式不执行 J1/J2 碰撞区限位，
	   该限位只在遥控模式中生效。 */
	limitPosition(&ArmData.J1.send_set, J1_Max_position, J1_Min_position); // 静态限位（防御性）：PC 模式不启用 J1/J2 碰撞区收紧
	limitPosition(&ArmData.J2.send_set, J2_Max_position, J2_Min_position);
	limitPosition(&ArmData.J3.send_set, J3_Max_position, J3_Min_position);
	limitPosition(&ArmData.J4.send_set, J4_Max_position, J4_Min_position);
	limitPosition(&ArmData.J5.send_set, J5_Max_position, J5_Min_position);
	limitPosition(&ArmData.J6.send_set, J6_Max_position, J6_Min_position);
	limitPosition(&ArmData.Grip.send_set, Grip_Max_position, Grip_Min_position);

	/* 同步斜坡状态，防止切出 PC 模式后跳变 */
	J1_L_position_set.out = ArmData.J1.send_set;
	J2_L_position_set.out = ArmData.J2.send_set;
	J3_L_position_set.out = ArmData.J3.send_set;
	J4_L_position_set.out = ArmData.J4.send_set;
	J5_L_position_set.out = ArmData.J5.send_set;
	J6_L_position_set.out = ArmData.J6.send_set;
	grip_L_position_set.out = ArmData.Grip.send_set;
}

/*************************************遥锟斤拷锟斤拷锟斤拷锟斤拷*******************************************/

/* Arm_L_Movement_J1_J4() —— 遥控模式下的主力程
 * 输入：VT13 遥控器通道 0..3（先经 rc_deadband_limit 去死区）
 * 映射：ch0 -> J1/J3，ch1 -> J2/J4，ch2 -> J5/J6，ch3 -> 夹爪
 * 流程：累加遥控增量 -> 增量限位（相对基准位置）-> 叠加零点/自定义控制器
 *       -> 硬限位 -> 写 send_set
 * 注意：本模式只有夹爪走 arm_ramp 斜坡，其余轴直接跟手；J1 的限位在
 *       J2 低于碰撞阈值时会收紧（见 J1_J2_ZONE_*）。
 */
void ARM_rc(void)
{
	int16_t channel_0,channel_1,channel_2,channel_3;
	/*锟斤拷取遥锟斤拷锟斤拷通锟斤拷锟斤拷锟诫，应锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷*/
/* 四个通道先过 ±10 的死区，避免摇杆回中抖动被累加进目标。 */
	rc_deadband_limit(RCData.rc.ch[0],channel_0,10);
	rc_deadband_limit(RCData.rc.ch[1],channel_1,10);
	rc_deadband_limit(RCData.rc.ch[2],channel_2,10);
	rc_deadband_limit(RCData.rc.ch[3],channel_3,10);
/* 每个控制周期都重写 PD 增益（数值是常量，确保从其他模式切回后仍正确）。 */
	/*锟斤拷锟斤拷锟斤拷PD锟斤拷值*/
	ArmData.J1.Kp = J1_PID_KP;ArmData.J1.Kd = J1_PID_KD;
	ArmData.J2.Kp = J2_PID_KP;ArmData.J2.Kd = J2_PID_KD;
	ArmData.J3.Kp = J3_PID_KP;ArmData.J3.Kd = J3_PID_KD;
	ArmData.J4.Kp = J4_PID_KP;ArmData.J4.Kd = J4_PID_KD;
	ArmData.J5.Kp = J5_PID_KP;ArmData.J5.Kd = J5_PID_KD;
	ArmData.J6.Kp = J6_PID_KP;ArmData.J6.Kd = J6_PID_KD;
	ArmData.Grip.Kp = Grip_PID_KP;ArmData.Grip.Kd = Grip_PID_KD;	
/* 夹爪斜坡步长 0.030 rad/拍：本模式只有夹爪限速，其余轴直接跟手。 */
	Grip_ramp = 0.030;							/* 本模式原来完全不设夹爪斜坡 */

    ArmData.J1.RC_Sum_Value += channel_0*0.00001; // 每拍角度增量 = 通道值 × 系数；ch0 同时驱动 J1/J3，ch1 同时驱动 J2/J4，ch2 驱动 J5/J6
	ArmData.J2.RC_Sum_Value += channel_1*0.00001;
	ArmData.J3.RC_Sum_Value += channel_0*0.00001;
	ArmData.J4.RC_Sum_Value += channel_1*0.00002; // J4 的累加系数是 J2 的两倍
	ArmData.J5.RC_Sum_Value += channel_2*0.00002;
	ArmData.J6.RC_Sum_Value += channel_2*0.00002;
	ArmData.Grip.RC_Sum_Value += channel_3*0.00005;
	/* 与最终目标使用同一模式入口基准，防止负位置切入后无法张开。 */
	limitPosition(&ArmData.Grip.RC_Sum_Value,
	              Grip_Max_position - ArmData.Grip.Initial_Total_position - ArmData.Grip.Key_Control_Value,
	              Grip_Min_position - ArmData.Grip.Initial_Total_position - ArmData.Grip.Key_Control_Value);

	
	/*锟斤拷锟酵革拷MIT锟斤拷锟斤拷模式锟铰的猴拷锟斤拷锟斤拷位锟斤拷*/
/* 目标位置 = 零点偏置 + 遥控累加值（J5/J6 再叠加自定义控制器微调）。 */
	J1_set_initial_data = ArmData.J1.Initial_Total_position + ArmData.J1.RC_Sum_Value;
	J2_set_initial_data = ArmData.J2.Initial_Total_position + ArmData.J2.RC_Sum_Value;
	J3_set_initial_data = ArmData.J3.Initial_Total_position + ArmData.J3.RC_Sum_Value;
	J4_set_initial_data = ArmData.J4.Initial_Total_position + ArmData.J4.RC_Sum_Value;
	J5_set_initial_data = ArmData.J5.Initial_Total_position + ArmData.J5.RC_Sum_Value + Custom_RC[4].DM_CC_receive;
	J6_set_initial_data = ArmData.J6.Initial_Total_position + ArmData.J6.RC_Sum_Value + Custom_RC[5].DM_CC_receive;
	Grip_set_initial_data = ArmData.Grip.Initial_Total_position + ArmData.Grip.RC_Sum_Value;

	J1_send_set = ArmData.J1.Initial_Total_position + ArmData.J1.RC_Sum_Value;
	J2_send_set = ArmData.J2.Initial_Total_position + ArmData.J2.RC_Sum_Value;
	J3_send_set = ArmData.J3.Initial_Total_position + ArmData.J3.RC_Sum_Value;
	J4_send_set = ArmData.J4.Initial_Total_position + ArmData.J4.RC_Sum_Value;
	J5_send_set = ArmData.J5.Initial_Total_position + ArmData.J5.RC_Sum_Value + Custom_RC[4].DM_CC_receive + ArmData.J5.Key_Control_Value;
	J6_send_set = ArmData.J6.Initial_Total_position + ArmData.J6.RC_Sum_Value + Custom_RC[5].DM_CC_receive + ArmData.J6.Key_Control_Value;
	Grip_send_set = ArmData.Grip.Initial_Total_position + ArmData.Grip.RC_Sum_Value + ArmData.Grip.Key_Control_Value;
	
	/*���λ�ø�ֵ*/
/* 把本拍目标写回 ArmData，供 Arm_Publish() 组装 MIT 帧下发。 */
	ArmData.J1.send_set = J1_send_set;
	ArmData.J2.send_set = J2_send_set;
	ArmData.J3.send_set = J3_send_set;
	ArmData.J4.send_set = J4_send_set;
	ArmData.J5.send_set = J5_send_set;
	ArmData.J6.send_set = J6_send_set;
	ArmData.Grip.send_set = Grip_send_set;

	/* 同步斜坡状态量到当前实际位置，避免切模式时从零点起跳 */
	J1_L_position_set.out = ArmData.J1.position;
	J2_L_position_set.out = ArmData.J2.position;
	J3_L_position_set.out = ArmData.J3.position;
	J4_L_position_set.out = ArmData.J4.position;
	J5_L_position_set.out = ArmData.J5.position;
	J6_L_position_set.out = ArmData.J6.position;
	/* 夹爪不同步斜坡：本模式夹爪要走自己的斜坡，见函数末尾 arm_ramp_calc */

//	Arm_Key_Control_mode();  //�������
	
/* 增量限位：把 RC_Sum_Value 限制在“基准位置 ± 行程”范围内。 */
	limitPosition(&ArmData.J1.RC_Sum_Value,J1_Max_position - ArmData.J1.Initial_Total_position,J1_Min_position - ArmData.J1.Initial_Total_position);
	limitPosition(&ArmData.J2.RC_Sum_Value,J2_Max_position - ArmData.J2.Initial_Total_position - ArmData.J2.Key_Control_Value,J2_Min_position - ArmData.J2.Initial_Total_position - ArmData.J2.Key_Control_Value);
	limitPosition(&ArmData.J3.RC_Sum_Value,J3_Max_position - ArmData.J3.Initial_Total_position,J3_Min_position - ArmData.J3.Initial_Total_position);
	limitPosition(&ArmData.J4.RC_Sum_Value,J4_Max_position - ArmData.J4.Initial_Total_position - ArmData.J4.Key_Control_Value,J4_Min_position - ArmData.J4.Initial_Total_position - ArmData.J4.Key_Control_Value);
	limitPosition(&ArmData.J5.RC_Sum_Value,J5_Max_position - ArmData.J5.Initial_Total_position,J5_Min_position - ArmData.J5.Initial_Total_position);
	limitPosition(&ArmData.J6.RC_Sum_Value,J6_Max_position - ArmData.J6.Initial_Total_position - ArmData.J6.Key_Control_Value,J6_Min_position - ArmData.J6.Initial_Total_position - ArmData.J6.Key_Control_Value);
	/* 夹爪的 RC_Sum 已经在上面用夹爪自己的限位钳过，这里不再重复 */

//				J1_send_set = R_J1_get_position_1;
//				J2_send_set = R_J2_get_position_1;
//				J3_send_set = R_J3_get_position_1;
//				J4_send_set = R_J4_get_position_1;
//				J5_send_set = R_J5_get_position_1;
//				J6_send_set = R_J6_get_position_1;
//				Grip_send_set =R_Grip_get_position_1;	
	limitPosition(&ArmData.J1.send_set,J1_Max_position,J1_Min_position); // J1 再走一次完整限位：前面收紧后的值仍在其区间内，不会被放宽
	limitPosition(&ArmData.J2.send_set,J2_Max_position,J2_Min_position);
	limitPosition(&ArmData.J3.send_set,J3_Max_position,J3_Min_position);
	limitPosition(&ArmData.J4.send_set,J4_Max_position,J4_Min_position);
	limitPosition(&ArmData.J5.send_set,J5_Max_position,J5_Min_position);
	limitPosition(&ArmData.J6.send_set,J6_Max_position,J6_Min_position);
	limitPosition(&ArmData.Grip.send_set,Grip_Max_position,Grip_Min_position);

	/* 夹爪走斜坡：本模式其它轴都是直接跟手，只有夹爪限速，避免夹伤/冲击 */
	arm_ramp_calc(&grip_L_position_set,Grip_ramp,ArmData.Grip.send_set); // 夹爪单轴斜坡：步长 Grip_ramp
	ArmData.Grip.send_set = grip_L_position_set.out;
	
/* 记录本拍实际下发值（仅调试回读用）。 */
	ArmData.J1.Now_set_position = ArmData.J1.send_set;
	ArmData.J2.Now_set_position = ArmData.J2.send_set;
	ArmData.J3.Now_set_position = ArmData.J3.send_set;
	ArmData.J4.Now_set_position = ArmData.J4.send_set;
	ArmData.J5.Now_set_position = ArmData.J5.send_set;
	ArmData.J6.Now_set_position = ArmData.J6.send_set;
	ArmData.Grip.Now_set_position = ArmData.Grip.send_set;
	
}

/* Arm_return_initial_position() —— 把机械臂平滑收回零点姿态
 * 特点：内部自带 osDelay(500) 与 do-while + osDelay(5) 的阻塞循环，会长时间占用
 *       Arm_L_Task（打断 8 ms 控制周期），因此主循环中当前并未调用它。
 * 流程：目标置为实测 -> 等 500 ms -> 刷新 position/基准 -> 重置斜坡 -> 下发 PD
 *       -> 以 0.002 rad/拍向零点收敛，直到 7 轴全部到位。
 */
void Arm_return_initial_position(void)
{
	ArmData.J1.send_set=joint_L_motor[0].position;
	ArmData.J2.send_set=joint_L_motor[1].position;
	ArmData.J3.send_set=joint_L_motor[2].position;
	ArmData.J4.send_set=joint_L_motor[3].position;
	ArmData.J5.send_set=joint_L_motor[4].position;
	ArmData.J6.send_set=joint_L_motor[5].position;
	ArmData.Grip.send_set=joint_L_motor[6].position;
	osDelay(500); // 阻塞等待 500 ms，让电机先跟上目标
	ArmData.J1.position=joint_L_motor[0].position;
	ArmData.J2.position=joint_L_motor[1].position;
	ArmData.J3.position=joint_L_motor[2].position;
	ArmData.J4.position=joint_L_motor[3].position;
	ArmData.J5.position=joint_L_motor[4].position;
	ArmData.J6.position=joint_L_motor[5].position;
	ArmData.Grip.position=joint_L_motor[6].position;

	ArmData.J1.send_set=joint_L_motor[0].position;
	ArmData.J2.send_set=joint_L_motor[1].position;
	ArmData.J3.send_set=joint_L_motor[2].position;
	ArmData.J4.send_set=joint_L_motor[3].position;
	ArmData.J5.send_set=joint_L_motor[4].position;
	ArmData.J6.send_set=joint_L_motor[5].position;
	ArmData.Grip.send_set=joint_L_motor[6].position;

/* 以当前实测位置刷新增量控制基准。 */
	ArmData.J1.Initial_Total_position=ArmData.J1.position;
	ArmData.J2.Initial_Total_position=ArmData.J2.position;	
	ArmData.J3.Initial_Total_position=ArmData.J3.position;
	ArmData.J4.Initial_Total_position=ArmData.J4.position;	
	ArmData.J5.Initial_Total_position=ArmData.J5.position;
	ArmData.J6.Initial_Total_position=ArmData.J6.position;	 
	ArmData.Grip.Initial_Total_position=ArmData.Grip.position;	

	arm_data_ramp_init(&J1_L_position_set,1,ArmData.J1.Initial_Total_position);	 // 斜坡状态以实测位置为起点（frame_period=1，语义见 user_lib.c）
	arm_data_ramp_init(&J2_L_position_set,1,ArmData.J2.Initial_Total_position); 
	arm_data_ramp_init(&J3_L_position_set,1,ArmData.J3.Initial_Total_position);	
	arm_data_ramp_init(&J4_L_position_set,1,ArmData.J4.Initial_Total_position); 
	arm_data_ramp_init(&J5_L_position_set,1,ArmData.J5.Initial_Total_position);	
	arm_data_ramp_init(&J6_L_position_set,1,ArmData.J6.Initial_Total_position);
	arm_data_ramp_init(&grip_L_position_set,1,ArmData.Grip.Initial_Total_position);
	
	ArmData.J1.Kp = J1_PID_KP;ArmData.J1.Kd = J1_PID_KD;
	ArmData.J2.Kp = J2_PID_KP;ArmData.J2.Kd = J2_PID_KD;
	ArmData.J3.Kp = J3_PID_KP;ArmData.J3.Kd = J3_PID_KD;
	ArmData.J4.Kp = J4_PID_KP;ArmData.J4.Kd = J4_PID_KD;
	ArmData.J5.Kp = J5_PID_KP;ArmData.J5.Kd = J5_PID_KD;
	ArmData.J6.Kp = J6_PID_KP;ArmData.J6.Kd = J6_PID_KD;
	ArmData.Grip.Kp = Grip_PID_KP;ArmData.Grip.Kd = Grip_PID_KD;
	
	fp32 J1_J2_J3_J4_J5_J6_J7_flag = 1; // 循环标志：1=仍在收拢，0=已全部到达零点

	do{	
		arm_ramp_calc(&J1_L_position_set,0.002,J1_zero_position); // 每拍步长 0.002 rad（约 0.4 rad/s）
		arm_ramp_calc(&J2_L_position_set,0.002,J2_zero_position);
		arm_ramp_calc(&J3_L_position_set,0.002,J3_zero_position);
		arm_ramp_calc(&J4_L_position_set,0.002,J4_zero_position);
		arm_ramp_calc(&J5_L_position_set,0.002,J5_zero_position);
		arm_ramp_calc(&J6_L_position_set,0.002,J6_zero_position);
		arm_ramp_calc(&grip_L_position_set,0.002,grip_zero_position);
		
		ArmData.J1.send_set = J1_L_position_set.out;
		ArmData.J2.send_set = J2_L_position_set.out;
		ArmData.J3.send_set = J3_L_position_set.out;
		ArmData.J4.send_set = J4_L_position_set.out;
		ArmData.J5.send_set = J5_L_position_set.out;
		ArmData.J6.send_set = J6_L_position_set.out;
		ArmData.Grip.send_set = grip_L_position_set.out;
		
		limitPosition(&ArmData.J1.send_set,J1_Max_position,J1_Min_position);
		limitPosition(&ArmData.J2.send_set,J2_Max_position,J2_Min_position);
		limitPosition(&ArmData.J3.send_set,J3_Max_position,J3_Min_position);
		limitPosition(&ArmData.J4.send_set,J4_Max_position,J4_Min_position);
		limitPosition(&ArmData.J5.send_set,J5_Max_position,J5_Min_position);
		limitPosition(&ArmData.J6.send_set,J6_Max_position,J6_Min_position);
		limitPosition(&ArmData.Grip.send_set,Grip_Max_position,Grip_Min_position);

		//�ж��Ƿ�ص���ʼ��̬
/* 7 轴斜坡输出全部等于零点即认为到位。 */
		if(J1_L_position_set.out == J1_zero_position && J2_L_position_set.out == J2_zero_position && J3_L_position_set.out == J3_zero_position && J4_L_position_set.out == J4_zero_position && J5_L_position_set.out == J5_zero_position && J6_L_position_set.out == J6_zero_position && grip_L_position_set.out == grip_zero_position)
		{
				J1_J2_J3_J4_J5_J6_J7_flag = 0;
		}
		osDelay(5); // 5 ms 一步，动作总耗时取决于最远轴的距离
	}
    while(J1_J2_J3_J4_J5_J6_J7_flag);
}

/* Chassis_Movement() —— 底盘运动时保持机械臂（当前为空实现，仅占位）。 */
void Chassis_Movement(void)
{
	
}

/*  锟斤拷位锟斤拷锟斤拷  */
/* limitPosition() —— 单值硬限位钳制
 * 参数：position      待钳位目标指针（就地修改）
 *       max_position  上限(rad)
 *       min_position  下限(rad)
 * 返回：无
 * 行为：>= 上限取上限，<= 下限取下限。所有 send_set / 累加器写回 ArmData 之前
 *       都先过一遍，防止超程撞机械限位。
 */
void limitPosition(fp32* position, fp32 max_position, fp32 min_position) 
{
    if (*position >= max_position)  // 超上限：直接截断（无减速包络）
    {
        *position = max_position;  // ����������ֵ�����������ֵ
    } 
    else if (*position <= min_position)  // 超下限
    {
        *position = min_position;  // ���������Сֵ����������Сֵ
    }
}

