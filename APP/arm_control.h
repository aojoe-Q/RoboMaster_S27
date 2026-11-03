#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

/*
* 模式仲裁与启动/恢复状态机（方案第 4、5 节）。
 *
* 本模块为纯逻辑，因此可在主机上做单元测试。读取输入源、电机反馈和输出
* 许可的板级胶水在 arm_output.c 中，每个控制周期调用一次
* ArmControl_Update()。本模块不写 Flash，也不产生任何电机力。
 * cycle. Nothing here programs Flash or produces motor force.
 */

#define ARM_CONTROL_AXES 7u

/* 恢复尝试前，反馈必须在这段时间内持续新鲜且合理。 */
#define ARM_CONTROL_SETTLE_MS 200u
/* 对齐请求超时未提交即取消，不重试。 */
#define ARM_CONTROL_ALIGN_TIMEOUT_MS 2000u
/* 恢复尝试若始终没有结论，不能一直卡住启动流程。 */
#define ARM_CONTROL_RESTORE_TIMEOUT_MS 3000u

/* 数值镜像 Arm_L_task.h 的 Arm_mode_t，修改时必须同步。 */
#define ARM_MODE_MANUAL_J1_J4 0u
#define ARM_MODE_POWERLESS    5u
#define ARM_MODE_TAKE_WATER   6u
#define ARM_MODE_CUSTOM       10u
#define ARM_MODE_ALIGN        11u

/* 位姿记录配置号：几何、限位或实测周期变化时递增，
   使旧记录被拒绝。 */
#define ARM_CONTROL_CONFIG_ID 1u

/* 与 DT7_DR16.h 中 KEY_PRESSED_OFFSET_Z 为同一位。 */
#define ARM_KEY_Z ((uint16_t)1u << 11)

typedef enum {
ARM_BOOT_LOCKED = 0,      /* 输出上锁，等待反馈 */
ARM_BOOT_WAIT_FEEDBACK,   /* 七轴反馈新鲜，等待稳定窗口 */
ARM_BOOT_RESTORE,         /* 判断保存的位姿是否可用 */
ARM_BOOT_READY_DISARMED,       /* 位姿已知，未显式进入模式前不出力 */
ARM_BOOT_NEED_REFERENCE,       /* 无可用记录：需要人工重新建立基准 */
ARM_BOOT_NEED_MASTER_ALIGN,    /* 主手映射/锚点不可用 */
ARM_BOOT_FAULT                 /* 故障锁存，直到复位 */
} ArmBootState;

typedef struct {
uint32_t source;      /* 拨杆来源：1 DT7，2 影子板 */
uint8_t keyboard_source; /* 键盘来源，编号同 arm_input.h */
bool online;      /* 所选控制源在线 */
uint8_t s[2];     /* s[0] 右拨杆，s[1] 左拨杆（RC_Right/RC_Left） */
uint16_t key;     /* 所选键盘位图 */
bool axes_ready;  /* 七轴反馈新鲜、有限且处于协议范围内 */
bool restore_ok;  /* 已恢复一条兼容的位姿记录 */
bool restore_known; /* 恢复尝试已有确定结论 */
bool fault;       /* 调用方锁存的电机或存储故障 */
} ArmControlInput;

typedef struct {
    ArmBootState state;
uint8_t mode;          /* Arm_mode_t 的取值 */
bool custom_latched;   /* Z 键锁存的自定义控制模式 */
bool align_pending;    /* 一个等待提交的对齐请求 */
bool align_active;     /* 对齐组合当前保持中 */
bool wait_idle;        /* 退出对齐后等待双中位 */
bool primed;           /* 首个样本不产生边沿 */
    bool z_prev;
    bool combo_prev;
    uint32_t state_tick;
    uint32_t align_tick;
uint32_t source;         /* 上一次拨杆来源，用于重新建立边沿 */
uint8_t keyboard_source; /* 上一次键盘来源 */
float offset[ARM_CONTROL_AXES]; /* boot_offset_rad，未实测前为 0 */
} ArmControl;

void ArmControl_Reset(ArmControl *c, uint32_t now);
void ArmControl_Update(ArmControl *c, const ArmControlInput *in, uint32_t now);
bool ArmControl_MotionAllowed(const ArmControl *c);
float ArmControl_AxisOffset(const ArmControl *c, unsigned axis);
/* 恢复公式（方案 3.2 节）：n = round((saved - raw) / period);
   offset = n * period。该式仅在每轴都有实测周期、且残差小于半个周期时
   才成功；否则位姿视为未知，机械臂继续保持无力。
   （周期 T 必须逐轴实测，禁止统一套 2π 或 2π/G。） */
bool ArmControl_SolveOffsets(const float saved[ARM_CONTROL_AXES],
                             const float raw[ARM_CONTROL_AXES],
                             const float period[ARM_CONTROL_AXES],
                             float offset[ARM_CONTROL_AXES]);

void ArmControl_RequestAlign(ArmControl *c, uint32_t now);
void ArmControl_AlignDone(ArmControl *c, bool ok, uint32_t now);
const char *ArmControl_StateName(ArmBootState state);

/* 板级胶水，实现在 arm_output.c 中（仅固件使用）。 */
void ArmControl_Init(uint32_t now);
void ArmControl_Step(uint32_t now);
uint8_t ArmControl_Mode(void);
ArmBootState ArmControl_BootState(void);
bool ArmControl_AlignPending(void);
void ArmControl_AlignComplete(bool ok);
float ArmControl_Offset(unsigned axis);

#endif /* ARM_CONTROL_H */
