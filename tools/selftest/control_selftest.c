#include "control_hal.h"
#include "arm_output.h"
#include "arm_motion.h"
#include "motor_encoding.h"
#include "can_health.h"
#include "bsp_can.h"
#include "DM_motor.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static Registers regs[3];
FDCAN_HandleTypeDef hfdcan1={&regs[0]},hfdcan2={&regs[1]},hfdcan3={&regs[2]};
joint_motor_measure_t joint_L_motor[7];
static uint32_t now, free_slots=8U, sends[7], enables[7], total, rx_count[3], decoded[3];
static uint8_t last[8], malformed, cancel_stuck;
static int fail_tx;
uint32_t xTaskGetTickCount(void) { return now; }
uint32_t xTaskGetTickCountFromISR(void) { return now; }
void vTaskDelayUntil(TickType_t *p,TickType_t n) { *p+=n; }
void osDelay(uint32_t n) { now+=n; }
TaskHandle_t xTaskGetCurrentTaskHandle(void) { return (void *)1; }
void vTaskNotifyGiveFromISR(TaskHandle_t t,BaseType_t *w) { (void)t; *w=0; }
void Error_Handler(void) { abort(); }
float uint_to_float(int x,float lo,float hi,int bits) { return lo+(float)x*(hi-lo)/(float)((1U<<bits)-1); }
static unsigned bus(FDCAN_HandleTypeDef *h) { return (unsigned)(h->Instance-regs); }
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *h,FDCAN_FilterTypeDef *f) { (void)h;(void)f;return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *h,uint32_t a,uint32_t b,uint32_t c,uint32_t d) { (void)h;(void)a;(void)b;(void)c;(void)d;return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *h,uint32_t a,uint32_t b) { (void)h;(void)a;(void)b;return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_ConfigTxDelayCompensation(FDCAN_HandleTypeDef *h,uint32_t a,uint32_t b) { (void)h;(void)a;(void)b;return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_EnableTxDelayCompensation(FDCAN_HandleTypeDef *h) { (void)h;return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *h) { (void)h;return HAL_OK; }
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *h) { (void)h;return free_slots; }
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *h,FDCAN_TxHeaderTypeDef *hdr,uint8_t *data)
{
    (void)h;
    if (fail_tx || !free_slots) return HAL_ERROR;
    free_slots--; total++;
    memcpy(last,data,8);
    if (hdr->Identifier>=1 && hdr->Identifier<=7) {
        unsigned j=hdr->Identifier-1;
        sends[j]++;
        if (data[0]==255 && data[6]==255 && data[7]==252) enables[j]++;
    }
    return HAL_OK;
}
HAL_StatusTypeDef canx_send_data(FDCAN_HandleTypeDef *h,uint16_t id,uint8_t *d,uint32_t len)
{
    FDCAN_TxFrame_TypeDef f={0}; f.hcan=h; f.Header.Identifier=id;
    memcpy(f.Data,d,len); return USER_FDCAN_AddMessageToTxFifoQ(&f);
}
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *h,uint32_t f) { (void)f;return rx_count[bus(h)]; }
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *h,uint32_t f,FDCAN_RxHeaderTypeDef *hdr,uint8_t *data)
{
    (void)f; rx_count[bus(h)]--;
    memset(hdr,0,sizeof(*hdr)); hdr->Identifier=1;hdr->DataLength=malformed?15:8;
    memset(data,0,malformed?64:8); data[0]=17;
    return HAL_OK;
}
void APP_CAN_RxHandler(FDCAN_HandleTypeDef *h,uint32_t id,uint8_t *d,uint32_t tick)
{ assert(id==1 && d[0]==17 && tick==now); decoded[bus(h)]++; }
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef *h,FDCAN_ErrorCountersTypeDef *ec)
{ (void)h;ec->TxErrorCnt=0;ec->RxErrorCnt=0;return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *h,uint32_t mask)
{ if (!cancel_stuck) h->Instance->TXBRP&=~mask;return HAL_OK; }
static void feedback(unsigned j) {
    joint_L_motor[j].motorid=j+1; joint_L_motor[j].status=1;
    joint_L_motor[j].update_count++; joint_L_motor[j].update_tick=now;
}
static void step(void) {
    Arm_Control_Snapshot c={0}; c.tick_ms=now; Arm_Control_Publish(&c);
    free_slots=8; Arm_Output_Step(now);
}
static void boot(void) {
    Arm_Output_Start();
    for (now=0;now<100;now+=4) {
        for (unsigned j=0;j<7;j++) if (enables[j]) feedback(j);
        step();
    }
    assert(Arm_Output_AllEnabled());
}
static void encoding(void) {
    assert(float_to_uint_clamped(-100,-12.5f,12.5f,16)==0);
    assert(float_to_uint_clamped(100,-12.5f,12.5f,16)==65535);
    for (int bits=1;bits<=16;bits++) {
        uint16_t prev=0;
        for (int i=-200;i<=200;i++) {
            uint16_t v=float_to_uint_clamped((float)i/10,-12.5f,12.5f,bits);
            assert(v>=prev && v<=((1U<<bits)-1)); prev=v;
        }
    }
    assert(float_to_uint_clamped(NAN,-1,1,16)==0);
    assert(float_to_uint_clamped(0,1,-1,16)==0);
    assert(float_to_uint_clamped(0,-1,1,17)==0);
    mit_ctrl(&hfdcan1,1,100,100,1000,100,100);
    for (unsigned i=0;i<8;i++) assert(last[i]==255);
    unsigned before=total;
    mit_ctrl(&hfdcan1,1,NAN,0,1,1,0);
    mit_ctrl(&hfdcan1,1,0,INFINITY,1,1,0);
    mit_ctrl(&hfdcan1,1,0,0,1,1,-INFINITY);
    assert(total==before && motor_tx_invalid[0]==3);
    float f=-1.2345f; uint32_t u=FloatTohex(f); assert(Hex_To_Float(&u,4)==f);
    fail_tx=1;mit_ctrl(&hfdcan1,2,0,0,1,1,0);
    assert(can_health[0].tx_drop==1 && can_joint_tx_drop[1]==1);
}
static void motion(void) {
    ArmMotion m={0}; float target=3;
    for (unsigned i=0;i<20000;i++) {
        if (i==700) target=-3;
        if (i==1400) target=0.1f;
        float dt=i%3==0?0.016f:0.008f;
        float p=m.position,v=m.velocity;
        Arm_Motion_Step(&m,target,i>=1000?0.1f:10.0f,dt);
        assert(fabsf(m.velocity)<=ARM_PC_VMAX_RAD_S+1e-6f);
        assert(fabsf(m.velocity-v)<=ARM_PC_AMAX_RAD_S2*dt+1e-6f);
        assert(fabsf(m.position-p)<=ARM_PC_VMAX_RAD_S*dt+1e-6f);
    }
    assert(fabsf(m.position-target)<1e-5f && fabsf(m.velocity)<1e-4f);
    float p=m.position;Arm_Motion_Step(&m,5,0.5f,10);
    assert(fabsf(m.position-p)<=0.5f*ARM_PC_DT_MAX_S);
}
static void enable(void) {
    /* Pre-existing feedback cannot confirm a new enable. */
    for (unsigned j=0;j<7;j++) feedback(j);
    Arm_Output_Start();step(); assert(!Arm_Output_AllEnabled());
    for (now=4;now<100;now+=4) {
        for (unsigned j=0;j<7;j++) if (enables[j]) feedback(j);
        step();
    }
    assert(Arm_Output_AllEnabled());
    for (;now<400;now+=4) { for (unsigned j=1;j<7;j++) feedback(j);step(); }
    assert(arm_output_diag.state[0]==EN_FAIL);
    unsigned stopped=sends[0];
    for (;now<500;now+=4) { for (unsigned j=1;j<7;j++) feedback(j);step(); }
    assert(sends[0]==stopped && arm_output_diag.offline_skip[0]>0);
    feedback(0);step();assert(arm_output_diag.state[0]==EN_FAIL); /* explicit rearm */
    Arm_Output_RequestRetry();now+=4;step();assert(arm_output_diag.state[0]==EN_SEND);
    now+=4;feedback(0);step();assert(arm_output_diag.state[0]==EN_OK);
}
static void retry(void) {
    Arm_Output_Start();
    for (now=0;now<1300;now+=4) step();
    for (unsigned j=0;j<7;j++) assert(arm_output_diag.state[j]==EN_FAIL && arm_output_diag.attempts[j]==50);
    unsigned old=total;
    for (;now<1500;now+=4) step();
    assert(total==old);
}
static void fifo(void) {
    boot(); unsigned before[7];memcpy(before,sends,sizeof(before));
    for (;now<380;now+=4) {
        for (unsigned j=0;j<7;j++) feedback(j);
        free_slots=2;Arm_Output_Step(now);
    }
    for (unsigned j=0;j<7;j++) assert(sends[j]>before[j] && arm_output_diag.fifo_defer[j]>0);
    assert(can_health[0].tx_drop==0);
}
static void bus_test(void) {
    boot();
    for (unsigned attempt=0;attempt<4;attempt++) {
        regs[0].PSR=FDCAN_PSR_BO;regs[0].CCCR=FDCAN_CCCR_INIT;regs[0].TXBRP=255;
        HAL_FDCAN_ErrorStatusCallback(&hfdcan1,FDCAN_IT_BUS_OFF);
        unsigned old=total;now+=4;step();assert(total==old);
        if (attempt==3) { assert(can_health[0].flags & CAN_HEALTH_RECOVERY_LIMIT);break; }
        assert(regs[0].TXBRP==0 && !(regs[0].CCCR & FDCAN_CCCR_INIT));
        regs[0].PSR=0;now+=4;step();
        assert(!can_health[0].blocked && arm_output_diag.bus_inhibit);
        Arm_Output_RequestRetry();now+=4;step();
        assert(!arm_output_diag.bus_inhibit);
    }
    assert(can_health[0].busoff_count==4 && can_health[0].recovery_count==3);
    /* Pending cancellation timeout must fail closed, not replay targets. */
    regs[1].PSR=FDCAN_PSR_BO;regs[1].CCCR=FDCAN_CCCR_INIT;regs[1].TXBRP=1;
    cancel_stuck=1;BSP_CAN_HealthStep(now);BSP_CAN_HealthStep(now+1001);
    assert(can_health[1].blocked && (can_health[1].flags & CAN_HEALTH_RECOVERY_TIMEOUT));
    assert(regs[1].CCCR & FDCAN_CCCR_INIT);
}
static void queue(void) {
    BSP_CAN_RxTaskInit();now=123;
    rx_count[0]=40;rx_count[1]=10;rx_count[2]=5;
    HAL_FDCAN_RxFifo0Callback(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    HAL_FDCAN_RxFifo1Callback(&hfdcan1,FDCAN_IT_RX_FIFO1_NEW_MESSAGE); /* wrong bus */
    HAL_FDCAN_RxFifo1Callback(&hfdcan2,FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
    HAL_FDCAN_RxFifo0Callback(&hfdcan3,FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    assert(rx_count[0]==0 && can_health[0].rx_overflow==9);
    BSP_CAN_ProcessRx();assert(decoded[0]==31 && decoded[1]==10 && decoded[2]==5);
    malformed=1;rx_count[0]=1;HAL_FDCAN_RxFifo0Callback(&hfdcan1,2);
    BSP_CAN_ProcessRx();assert(decoded[0]==31 && can_health[0].rx_invalid==1);
}
int main(int argc,char **argv) {
    assert(argc==2);
    if (!strcmp(argv[1],"encoding")) encoding();
    else if (!strcmp(argv[1],"motion")) motion();
    else if (!strcmp(argv[1],"enable")) enable();
    else if (!strcmp(argv[1],"retry")) retry();
    else if (!strcmp(argv[1],"fifo")) fifo();
    else if (!strcmp(argv[1],"bus")) bus_test();
    else if (!strcmp(argv[1],"queue")) queue();
    else return 1;
    printf("%s: OK\n",argv[1]);return 0;
}
