/* Host fakes around extracted production control functions. */
#include <assert.h>
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#undef assert
#define assert(c) do { if (!(c)) { fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,#c); exit(1); } } while(0)
#include "arm_output.h"
typedef uint32_t TickType_t;
#define portTICK_PERIOD_MS 1U
#define pdMS_TO_TICKS(x) (x)
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define RC_Right 0
#define RC_Left 1
#define switch_is_up(s) ((s)==1)
#define switch_is_mid(s) ((s)==3)
#define switch_is_down(s) ((s)==2)
#define rc_deadband_limit(i,o,d) ((o)=((i)>(d)||(i)<-(d))?(i):0)
ArmData_t ArmData;
Custom_controller_t Custom_RC[6];
struct { struct { int16_t ch[5]; uint8_t s[2]; } rc; } RCData;
struct { float position; } joint_L_motor[7];
volatile uint8_t Arm_L_Ready = 1;
ArmOutputDiagnostics arm_output_diag;
static Arm_Control_Snapshot arm_pending_command, captured;
static uint8_t grip_torque_mode, retry_powerless_selected;
static float s_grip_blend;
static unsigned loops, retries, publications;
static uint32_t now;
static const char *scenario;
static jmp_buf exit_loop;
ramp_function_source_t J1_L_position_set, J2_L_position_set, J3_L_position_set,
    J4_L_position_set, J5_L_position_set, J6_L_position_set, grip_L_position_set;
static ramp_function_source_t *const arm_joint_ramp[7] = {
    &J1_L_position_set, &J2_L_position_set, &J3_L_position_set,
    &J4_L_position_set, &J5_L_position_set, &J6_L_position_set,
    &grip_L_position_set
};
fp32 J1_send_set,J2_send_set,J3_send_set,J4_send_set,J5_send_set,J6_send_set,Grip_send_set;
fp32 J1_set_initial_data,J2_set_initial_data,J3_set_initial_data,J4_set_initial_data,
    J5_set_initial_data,J6_set_initial_data,Grip_set_initial_data;
double J1_ramp,J2_ramp,J3_ramp,J4_ramp,J5_ramp,J6_ramp,Grip_ramp;
static DM_contorl_t *const arm_joint_control[7] = {
    &ArmData.J1,&ArmData.J2,&ArmData.J3,&ArmData.J4,&ArmData.J5,&ArmData.J6,&ArmData.Grip
};
uint8_t GetArmMode(void);
uint32_t xTaskGetTickCount(void) { return now; }
void Arm_Control_Publish(const Arm_Control_Snapshot *c) { captured=*c; publications++; }
void Arm_Output_RequestRetry(void) {
    /* Model immediate preemption by the output task at the request call. */
    assert(publications == loops+1 && captured.tick_ms == now);
    for(unsigned j=0;j<7;j++)
        assert(captured.kp[j]==0 && captured.kd[j]==0 && captured.torque[j]==0);
    retries++;
}
void Arm_USB_PC_Disarm(void) {}
void Arm_PC_Control_Enter(void) {}
void Arm_PC_Control(void) {}
void Arm_L_Init(void) {}
void data_updata(void) {
    uint8_t right=2, left=1;
    if (!strcmp(scenario,"bus_hold") || !strcmp(scenario,"explicit") || !strcmp(scenario,"reconnect")) {
        if (loops>=4) arm_output_diag.bus_inhibit=1;
    }
    if ((!strcmp(scenario,"explicit") || !strcmp(scenario,"reconnect")) && loops>=6) left=2;
    if (!strcmp(scenario,"reconnect") && loops>=10 && loops<13) right=left=0;
    if (!strcmp(scenario,"loss") && loops>=4 && loops<8) right=left=0;
    if (!strcmp(scenario,"invalid") && loops>=4) { right=4;left=2; }
    if (!strcmp(scenario,"debounce") && (loops==5 || loops==8 || loops==9)) left=2;
    if (!strcmp(scenario,"grip_retry")) { right=loops<6?1:2;left=loops<6?3:2; }
    if (!strcmp(scenario,"unready")) { right=left=2;Arm_L_Ready=loops>=6; }
    RCData.rc.s[0]=right;RCData.rc.s[1]=left;
    ArmData.ArmMode=GetArmMode();
}
void vTaskDelayUntil(TickType_t *t,TickType_t n) {
    if (loops==3 && strcmp(scenario,"grip_retry") && strcmp(scenario,"unready")) {
        assert(captured.kp[1]>0);
        assert(fabsf(captured.pos[1]+1.5f)<1e-5f);
    }
    if (!strcmp(scenario,"grip_retry") && loops==5) assert(captured.torque[6]>0);
    if ((arm_output_diag.bus_inhibit || !RCData.rc.s[0] || RCData.rc.s[0]>3) && loops>=4) {
        for(unsigned j=0;j<7;j++) assert(captured.kp[j]==0 && captured.kd[j]==0 && captured.torque[j]==0);
    }
    *t+=n;now+=n;
    if (++loops==20) longjmp(exit_loop,1);
}
/* REAL_FUNCTIONS */
int main(int argc,char **argv) {
    assert(argc==2);scenario=argv[1];
    grip_L_position_set.frame_period=1.0f; /* Arm_L_Init's ramp setup */
    ArmData.J2.position=-1.5f;ArmData.Grip.position=-6.0f;
    Arm_Powerless();
    if (!strcmp(scenario,"grip_limits")) {
        RCData.rc.ch[3]=660;
        for(unsigned n=0;n<100;n++) {
            Arm_L_Movement_J1_J4();ArmData.Grip.position=ArmData.Grip.send_set;
        }
        assert(ArmData.Grip.send_set>-5.9f && ArmData.Grip.RC_Sum_Value>0);
        /* Exercise both end stops and a nonzero keyboard offset. */
        ArmData.Grip.Key_Control_Value=0.25f;
        for(unsigned n=0;n<3000;n++) {
            Arm_L_Movement_J1_J4();ArmData.Grip.position=ArmData.Grip.send_set;
        }
        assert(fabsf(ArmData.Grip.send_set-Grip_Max_position)<1e-4f);
        RCData.rc.ch[3]=-660;
        for(unsigned n=0;n<3000;n++) {
            Arm_L_Movement_J1_J4();ArmData.Grip.position=ArmData.Grip.send_set;
        }
        assert(fabsf(ArmData.Grip.send_set-Grip_Min_position)<1e-4f);
        RCData.rc.ch[3]=660;Arm_L_Movement_J1_J4();
        assert(ArmData.Grip.send_set>Grip_Min_position);
    } else {
        if (!setjmp(exit_loop)) Arm_L_Task();
        unsigned expected=(!strcmp(scenario,"explicit") || !strcmp(scenario,"reconnect") || !strcmp(scenario,"grip_retry"));
        assert(retries==expected);
    }
    printf("%s: OK\n",scenario);
    return 0;
}
