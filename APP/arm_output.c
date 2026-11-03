#include "arm_output.h"
#include "Arm_L_task.h"
#include "DM_motor.h"
#include "fdcan.h"
#include "arm_input.h"
#include "arm_control.h"
#include "multi_turn_store.h"
#include <string.h>
#include <math.h>
volatile ArmOutputReason arm_output_reason=ARM_NEED_REFERENCE;
static ArmGate gate; /* 零初始化：没有基准、没有配置、没有许可。 */
static uint32_t lock(void) { uint32_t p=__get_PRIMASK(); __disable_irq(); return p; }
static void unlock(uint32_t p) { __set_PRIMASK(p); }
/* 输出许可当前是否关闭（关闭即视为上锁）。 */
bool ArmOutput_IsLocked(void) { return !gate.permitted; }
/* 存储层报告 ECC 或读写故障：锁存原因并撤销输出许可。 */
void ArmOutput_StorageFault(void)
{
    gate.permitted=false; gate.epoch++; arm_output_reason=ARM_STORAGE_FAULT;
}
/* 电机反馈入口：反馈过期或状态异常时清除基准并撤销许可。 */
void ArmOutput_Feedback(unsigned axis,float raw,uint8_t state,uint32_t tick)
{
    uint32_t p=lock();
    if(axis<7) {
        ArmAxisFeedback *f=&gate.feedback[axis];
        if(f->valid && ((uint32_t)(tick-f->tick)>100 || state!=1)) {
            f->referenced=false;
            if(gate.permitted) ArmGate_Revoke(&gate);
        }
        f->raw=raw; f->state=state; f->tick=tick; f->valid=true;
    }
    unlock(p);
}
/* 每个控制周期调用：把 ArmData 组装成命令帧，并刷新输入来源与许可。 */
void ArmOutput_Publish(void)
{
    ArmCommandFrame next={0};
    uint32_t p;
    /* 只有 Arm 任务写 ArmData，这里把已完成的帧原子地发布出去。 */
    next.axis[0].position=ArmData.J1.send_set;
    next.axis[0].kp=ArmData.J1.Kp; next.axis[0].kd=ArmData.J1.Kd;
    next.axis[1].position=ArmData.J2.send_set;
    next.axis[1].kp=ArmData.J2.Kp; next.axis[1].kd=ArmData.J2.Kd;
    next.axis[2].position=ArmData.J3.send_set;
    next.axis[2].kp=ArmData.J3.Kp; next.axis[2].kd=ArmData.J3.Kd;
    next.axis[3].position=ArmData.J4.send_set;
    next.axis[3].kp=ArmData.J4.Kp; next.axis[3].kd=ArmData.J4.Kd;
    next.axis[4].position=ArmData.J5.send_set;
    next.axis[4].kp=ArmData.J5.Kp; next.axis[4].kd=ArmData.J5.Kd;
    next.axis[5].position=ArmData.J6.send_set;
    next.axis[5].kp=ArmData.J6.Kp; next.axis[5].kd=ArmData.J6.Kd;
    next.axis[6].position=ArmData.Grip.send_set;
    next.axis[6].kp=ArmData.Grip.Kp; next.axis[6].kd=ArmData.Grip.Kd;
    ArmInputSnapshot input;
    ArmInput_Read(&input);
    next.tick=HAL_GetTick();
    p=lock();
    if(gate.source!=input.source || !input.online) {
        if(gate.permitted) ArmGate_Revoke(&gate);
    }
    gate.source=input.source;
    gate.input_valid=input.online;
    gate.input_tick=((uint32_t)(next.tick-input.switches_tick)>(uint32_t)(next.tick-input.keyboard_tick))?input.switches_tick:input.keyboard_tick;
    next.epoch=gate.epoch; next.seq=gate.command.seq+1; next.source=gate.source;
    gate.command=next;
    unlock(p);
}
/* 每轴下发入口：先过门控的整臂校验，再决定出力或零增益失能。 */
void ArmOutput_Send(unsigned axis)
{
    ArmAxisCommand out;
    uint32_t p=lock();
    bool allowed=ArmGate_Select(&gate,axis,HAL_GetTick(),&out);
    /* 首次集成刻意不提供使能过渡：被拒绝的帧后面跟的是失能，
       而不是会产生力的回退输出。 */
    mit_ctrl(&hfdcan1,(uint16_t)(axis+1),out.position,out.velocity,out.kp,out.kd,out.torque);
    if (!allowed) unenable_motor_mode(&hfdcan1,(uint16_t)(axis+1),0);
    unlock(p);
}
/* 判断是否处于可以安全擦写 Flash 的停机状态。 */
bool ArmOutput_Stopped(void)
{
    unsigned i; bool stopped=true;
    uint32_t p=lock(), now=HAL_GetTick();
    if(gate.permitted || hfdcan1.Instance->TXBRP!=0) stopped=false;
    for(i=0;i<7;i++)
        if(!gate.feedback[i].valid || gate.feedback[i].state!=0 ||
           (uint32_t)(now-gate.feedback[i].tick)>100) stopped=false;
    unlock(p);
    return stopped;
}

/* ---- 模式仲裁与启动/恢复状态（方案第 4、5 节） ------------------------- */

static ArmControl control;

/* 各关节转一圈的物理周期（关节坐标系下的弧度）。0 表示"尚未实测"：
   此时启动流程停在 NEED_REFERENCE，而不是去猜一个周期（方案 3.2 节）。 */

static const float axis_period[ARM_CONTROL_AXES]={0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
static bool restore_ok,restore_done;
static unsigned restore_attempts;
static MT_Store restore_store;

static void restore_from_log(void)
{
    float raw[ARM_CONTROL_AXES],offset[ARM_CONTROL_AXES];
    const MT_Record *r;
    unsigned i;
    uint32_t p;

    MT_Result result;

    if(restore_attempts>=500) { restore_done=true; return; }
    restore_attempts++;
    result=MT_H7_Open(&restore_store,ARM_CONTROL_CONFIG_ID);
    if(result==MT_LOCKED) return;   /* 维护窗口占用中，稍后重试 */
    restore_done=true;              /* 已有定论，不再重试 */
    if(result!=MT_OK) return;
    r=&restore_store.latest;
    if((r->slave_valid_mask&0x7Fu)!=0x7Fu) return;

    p=lock();
    for(i=0;i<ARM_CONTROL_AXES;i++) raw[i]=gate.feedback[i].raw;
    unlock(p);

    if(!ArmControl_SolveOffsets(r->slave_logical_pos,raw,axis_period,offset)) return;
    for(i=0;i<ARM_CONTROL_AXES;i++) control.offset[i]=offset[i];
    restore_ok=true;
}

static bool feedback_ready(uint32_t now, bool *fault)
{
    const ArmAxisFeedback *f;
    unsigned i;
    bool ready=true;
    for(i=0;i<7;i++) {
        f=&gate.feedback[i];
        if(!f->valid || (uint32_t)(now-f->tick)>100) ready=false;
        if(f->valid && f->state>=2) *fault=true;
        if(!isfinite(f->raw) || f->raw<-12.5f || f->raw>12.5f) ready=false;
    }
    return ready;
}

/* 只有启动状态已知位姿、且所选模式要求出力时，门控才会打开；
   每次下发仍要在 ArmGate_Select 中重新校验全部七轴。 */
static void apply_reference(void)
{
    static const float minimum[ARM_CONTROL_AXES]={
        J1_Min_position,J2_Min_position,J3_Min_position,J4_Min_position,
        J5_Min_position,J6_Min_position,Grip_Min_position};
    static const float maximum[ARM_CONTROL_AXES]={
        J1_Max_position,J2_Max_position,J3_Max_position,J4_Max_position,
        J5_Max_position,J6_Max_position,Grip_Max_position};
    bool allowed=ArmControl_MotionAllowed(&control);
    bool referenced=(control.state==ARM_BOOT_READY_DISARMED);
    unsigned i;
    uint32_t p=lock();
    for(i=0;i<ARM_CONTROL_AXES;i++) {
        gate.feedback[i].offset=control.offset[i];
        gate.feedback[i].minimum=minimum[i];
        gate.feedback[i].maximum=maximum[i];
        gate.feedback[i].referenced=referenced;
    }
    gate.configured=true;
    if(allowed) {
        if(!gate.permitted) { gate.epoch++; gate.permitted=true; }
    } else if(gate.permitted) {
        ArmGate_Revoke(&gate);
    }
    unlock(p);
}

/* 仅在门控打开期间使能电机，且每次使能后紧跟一帧"实测位置 + 零增益"，
   因此单纯使能不会产生任何力矩。离开门控时重新失能全部七轴。 */

static bool outputs_enabled;

static void update_power(bool allow)
{
    unsigned i;
    uint32_t p;
    float raw;

    if(allow==outputs_enabled) return;
    if(allow) {
        for(i=0;i<ARM_CONTROL_AXES;i++) {
            p=lock();
            raw=gate.feedback[i].raw;
            unlock(p);
            enable_motor_mode(&hfdcan1,(uint16_t)(i+1),0);
            mit_ctrl(&hfdcan1,(uint16_t)(i+1),raw,0.0f,0.0f,0.0f,0.0f);
        }
    } else {
        for(i=0;i<ARM_CONTROL_AXES;i++) unenable_motor_mode(&hfdcan1,(uint16_t)(i+1),0);
    }
    outputs_enabled=allow;
}

/* 板级初始化：清状态，并建立第一份基准与限位配置。 */
void ArmControl_Init(uint32_t now)
{
    ArmControl_Reset(&control,now);
    apply_reference();
}

/* 周期步进：读输入与反馈、推进仲裁，并按许可使能或失能电机。 */
void ArmControl_Step(uint32_t now)
{
    ArmControlInput in;
    ArmInputSnapshot input;
    uint32_t p;
    bool fault=(arm_output_reason==ARM_STORAGE_FAULT);
    bool ready;

    ArmInput_Read(&input);

    if(control.state==ARM_BOOT_RESTORE && !restore_done) restore_from_log();

    p=lock();
    ready=feedback_ready(now,&fault);
    unlock(p);

    in.source=input.source;
    in.keyboard_source=input.keyboard_source;
    in.online=input.online;
    in.s[0]=input.s[0];
    in.s[1]=input.s[1];
    in.key=input.key;
    in.axes_ready=ready;
    in.fault=fault;
    /* restore_known 保持 false 直到存储给出确定结论，
       避免维护窗口忙时被静默当成 NEED_REFERENCE。 */
    in.restore_known=restore_done;
    in.restore_ok=restore_ok;

    ArmControl_Update(&control,&in,now);
    apply_reference();
    update_power(ArmControl_MotionAllowed(&control));
}

/* 当前仲裁出的模式（Arm_mode_t 取值）。 */
uint8_t ArmControl_Mode(void)
{
    return control.mode;
}

/* 当前启动状态。 */
ArmBootState ArmControl_BootState(void)
{
    return control.state;
}

/* 是否有一次尚未提交的对齐请求。 */
bool ArmControl_AlignPending(void)
{
    return control.align_pending;
}

/* 对齐事务结束通知：成功则保持，失败转 NEED_MASTER_ALIGN。 */
void ArmControl_AlignComplete(bool ok)
{
    ArmControl_AlignDone(&control,ok,HAL_GetTick());
    apply_reference();
}

float ArmControl_Offset(unsigned axis)
{
    return ArmControl_AxisOffset(&control,axis);
}
