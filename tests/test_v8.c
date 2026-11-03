#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "arm_gate.h"
#include "arm_control.h"
#include "multi_turn_store.h"
#include "master_link.h"
#include "Image_Transmission.h"
#include "CRC.h"
#include "usart.h"
UART_HandleTypeDef huart1;
static uint32_t tick=1;
uint32_t HAL_GetTick(void) { return tick; }
static uint8_t flash_mem[2*MT_SECTOR_SIZE];
static int cut=-1,write_calls,erase_calls,read_error,locked;
static bool rd(void *c,uint32_t a,uint8_t *p,uint32_t n)
{ (void)c; assert(a+n<=sizeof(flash_mem)); if(read_error)return false; memcpy(p,flash_mem+a,n);return true; }
static bool wr(void *c,uint32_t a,const uint8_t *p)
{
    unsigned i; (void)c; assert(a%32==0&&a+32<=sizeof(flash_mem)); write_calls++;
    for(i=0;i<32;i++) { assert(flash_mem[a+i]==255); if(cut==0)return false; if(cut>0)cut--; flash_mem[a+i]=p[i]; }
    return true;
}
static bool er(void *c,unsigned s)
{ (void)c; assert(s<2); erase_calls++; memset(flash_mem+s*MT_SECTOR_SIZE,255,MT_SECTOR_SIZE); return true; }
static bool maint(void *c) { (void)c; return !locked; }
static MT_Backend io={0,rd,wr,er,maint};
static MT_Record record(void)
{
    MT_Record r={0}; unsigned i;
    r.config_id=123; r.slave_valid_mask=127;
    r.slave_logical_pos[0]=5.2f;
    r.master_wire_int[0]=INT32_MIN; r.master_wire_int[1]=INT32_MAX;
    for(i=0;i<6;i++) r.master_anchor[i]=2147483647.0+i;
    for(i=0;i<30;i++) r.master_payload[i]=(uint8_t)i;
    return r;
}
static void storage_tests(void)
{
    MT_Store s,t; MT_Record r=record(),decoded; uint8_t b[288],old[288];
    int stop; unsigned i;
    assert(MT_CRC32((const uint8_t *)"123456789",9)==0xcbf43926U);
    MT_Encode(&r,b); assert(MT_Decode(b,&decoded));
    assert(decoded.master_wire_int[0]==INT32_MIN&&decoded.master_wire_int[1]==INT32_MAX);
    assert(decoded.master_anchor[5]==2147483652.0);
    assert(b[142]==255&&b[143]==255&&b[208]==255&&b[251]==255&&b[272]==255);
    b[35]^=1; assert(!MT_Decode(b,&decoded));
    memset(flash_mem,255,sizeof(flash_mem));
    assert(MT_Scan(&s,&io,123)==MT_EMPTY);
    locked=1; assert(MT_Append(&s,&r,false)==MT_LOCKED); assert(write_calls==0);
    locked=0; assert(MT_Append(&s,&r,false)==MT_OK);
    memcpy(old,flash_mem,288);
    /* 每写入一个字节后都断电一次，包括全部 32 个提交字节。
       真正带 ECC 错误的字另外用读错误用例测试。 */
    for(stop=0;stop<288;stop++) {
        memset(flash_mem,255,sizeof(flash_mem));memcpy(flash_mem,old,288);
        assert(MT_Scan(&s,&io,123)==MT_OK);
        r.slave_logical_pos[0]=6.2f;cut=stop;
        assert(MT_Append(&s,&r,false)==MT_IO);cut=-1;
        assert(MT_Scan(&t,&io,123)==MT_OK);
        assert(t.latest.seq==1||t.latest.seq==2);
        assert(t.latest.slave_logical_pos[0]==(t.latest.seq==1?5.2f:6.2f));
        assert(!memcmp(flash_mem,old,288));
        assert(MT_Append(&t,&r,false)==MT_OK); /* 不重用撕裂的槽 */
    }
    memset(flash_mem,255,sizeof(flash_mem));
    assert(MT_Scan(&s,&io,123)==MT_EMPTY);
    for(i=0;i<MT_SLOTS;i++) assert(MT_Append(&s,&r,false)==MT_OK);
    memcpy(old,flash_mem+(MT_SLOTS-1)*288,288);
    assert(MT_Append(&s,&r,false)==MT_FULL);
    assert(MT_Append(&s,&r,true)==MT_OK);
    assert(erase_calls==1&&!memcmp(old,flash_mem+(MT_SLOTS-1)*288,288));
    assert(MT_Scan(&s,&io,123)==MT_OK&&s.latest.seq==456&&s.active==1);
    assert(MT_Scan(&s,&io,124)==MT_INCOMPATIBLE&&!s.scanned);
    read_error=1; assert(MT_Scan(&s,&io,123)==MT_IO&&!s.scanned);read_error=0;
    /* 序号回绕；最新的不兼容格式必须挡住更旧的好记录。 */
    memset(flash_mem,255,sizeof(flash_mem));r.seq=UINT32_MAX;MT_Encode(&r,flash_mem);
    r.seq=0;MT_Encode(&r,flash_mem+288);
    assert(MT_Scan(&s,&io,123)==MT_OK&&s.latest.seq==0);
    flash_mem[288+4]++;
    { uint32_t crc=MT_CRC32(flash_mem+288,252); unsigned j;
      for(j=0;j<4;j++){flash_mem[288+252+j]=(uint8_t)(crc>>(j*8));flash_mem[288+264+j]=(uint8_t)(crc>>(j*8));} }
    assert(MT_Scan(&s,&io,123)==MT_INCOMPATIBLE);
}
static ArmGate ready(void)
{
    ArmGate g={0}; unsigned i;
    g.configured=g.permitted=g.input_valid=true;g.input_tick=100;
    g.epoch=g.command.epoch=2;g.source=g.command.source=1;g.command.tick=100;
    for(i=0;i<7;i++) {
        g.feedback[i].valid=g.feedback[i].referenced=true;
        g.feedback[i].state=1;g.feedback[i].tick=100;
        g.feedback[i].minimum=-20;g.feedback[i].maximum=20;
        g.command.axis[i].kp=20;g.command.axis[i].kd=1;
    }
    return g;
}
static void gate_tests(void)
{
    ArmGate g={0}; ArmAxisCommand c; unsigned i;
    for(i=0;i<7;i++){assert(!ArmGate_Select(&g,i,100,&c));assert(c.kp==0&&c.kd==0&&c.torque==0);}
    g=ready();g.feedback[0].raw=-1.0831853f;g.feedback[0].offset=6.2831853f;g.command.axis[0].position=5.2f;
    assert(ArmGate_Select(&g,0,100,&c)&&fabsf(c.position+1.0831853f)<0.00001f);
    ArmGate_Revoke(&g);g.permitted=true;
    assert(!ArmGate_Select(&g,6,100,&c)); /* 旧代次绝不复活 */
    for(i=0;i<7;i++) {
        g=ready();g.feedback[i].state=8;
        assert(!ArmGate_Select(&g,6,100,&c)&&c.torque==0);
        g=ready();g.command.axis[i].position=NAN;
        assert(!ArmGate_Select(&g,0,100,&c));
    }
    g=ready();assert(!ArmGate_Select(&g,0,151,&c)&&!g.permitted);
    g=ready();g.source=2;assert(!ArmGate_Select(&g,0,100,&c));
    g=ready();g.maintenance=true;g.command.axis[6].torque=0.2f;
    assert(!ArmGate_Select(&g,6,100,&c)&&c.torque==0);
    g=ready();g.feedback[0].offset=13;assert(!ArmGate_Select(&g,0,100,&c));
    g=ready();assert(ArmGate_Select(&g,0,100,&c));g.input_valid=false;
    assert(!ArmGate_Select(&g,1,100,&c));
    g=ready();g.input_tick=g.command.tick=UINT32_MAX-10;
    for(i=0;i<7;i++)g.feedback[i].tick=UINT32_MAX-10;
    assert(ArmGate_Select(&g,6,10,&c)); /* HAL tick 回绕 */
}
static unsigned frame(uint8_t *b,unsigned id,unsigned payload)
{
    unsigned i,n=payload+9;
    memset(b,0,n);b[0]=0xa5;b[1]=(uint8_t)payload;b[3]=17;Append_CRC8_Check_Sum(b,5);
    b[5]=(uint8_t)id;b[6]=(uint8_t)(id>>8);
    for(i=0;i<payload;i++)b[i+7]=(uint8_t)(i+1);
    Append_CRC16_Check_Sum(b,n);return n;
}
static void parser_tests(void)
{
    uint8_t b[100],k[39];unsigned n,i,j;MasterRawSnapshot m;ImageKeyboardSnapshot key;
    Image_Transmission_StreamReset();frame(b,0x0302,30);
    for(i=0;i<39;i++)Image_Transmission_Info_Update(b+i,1);
    assert(MasterLink_Read(&m)&&m.seq==1&&m.payload[29]==30);
    n=m.seq;
    frame(k,0x0304,12);tick=77;
    Image_Transmission_Info_Update(k,21);
    assert(ImageKeyboard_Read(&key)&&key.rx_tick==77);
    MasterLink_Read(&m);assert(m.rx_tick==1&&m.seq==n);
    frame(b,0x0303,30);Image_Transmission_Info_Update(b,39);
    MasterLink_Read(&m);assert(m.seq==n);
    frame(b,0x0302,30);b[38]^=1;Image_Transmission_Info_Update(b,39);
    MasterLink_Read(&m);assert(m.seq==n);
    b[0]=0xa5;b[1]=255;b[2]=255;Append_CRC8_Check_Sum(b,5);
    Image_Transmission_Info_Update(b,5);
    frame(b,0x0302,30);Image_Transmission_Info_Update(b,39);
    MasterLink_Read(&m);assert(m.seq==n+1);
    /* 所有可能的切分、噪声、粘包，以及重复的未决事件。 */
    for(i=0;i<=39;i++) {
        Image_Transmission_StreamReset();
        Image_Transmission_Info_Update((uint8_t *)"noise",5);
        Image_Transmission_Info_Update(b,i);Image_Transmission_Info_Update(b+i,39-i);
        assert(MasterLink_Read(&m));
    }
    Image_Transmission_StreamReset();n=m.seq;
    for(i=0;i<1000;i++) {
        for(j=0;j<39;j++) {
            static unsigned pos;
            ((uint8_t *)Image_Trans_MultiRx_Buff)[pos]=b[j];pos=(pos+1)%256;
            if(pos%128==0)Image_Transmission_DMAEvent(pos);
            /* 先 IDLE 再补发的 HT/TC：重放游标必须是空操作。 */
            if(j==38){Image_Transmission_DMAEvent(pos);Image_Transmission_DMAEvent(pos);}
        }
    }
    MasterLink_Read(&m);assert(m.seq==n+1000);
    /* 保留 VT13 通道；损坏和不完整的 21 字节包一律拒绝。 */
    memset(k,0,21);k[0]=0xa9;k[1]=0x53;k[17]=0x34;k[18]=0x12;Append_CRC16_Check_Sum(k,21);
    Image_Transmission_Info_Update(k,10);Image_Transmission_Info_Update(k+10,11);
    assert(VT13_Info.Key.V==0x1234);
    MasterLink_Invalidate();assert(!MasterLink_Read(&m)&&!ImageKeyboard_Read(&key));
}
#define SW_UP   1u
#define SW_DOWN 2u
#define SW_MID  3u

static ArmControlInput input(uint8_t right,uint8_t left,uint16_t key)
{
    ArmControlInput in={0};
    in.online=true; in.s[0]=right; in.s[1]=left; in.key=key;
    in.axes_ready=true; in.restore_ok=false; in.restore_known=true; in.fault=false;
    return in;
}

static void control_tests(void)
{
    ArmControl c; ArmControlInput in; uint32_t t=1000;

    /* 启动：反馈缺失时不出力，稳定后给出结论。 */
    ArmControl_Reset(&c,t);
    in=input(SW_MID,SW_MID,0);
    in.axes_ready=false;
    ArmControl_Update(&c,&in,t);
    assert(c.state==ARM_BOOT_LOCKED&&c.mode==ARM_MODE_POWERLESS);
    assert(!ArmControl_MotionAllowed(&c));
    in.axes_ready=true;
    ArmControl_Update(&c,&in,t);
    assert(c.state==ARM_BOOT_WAIT_FEEDBACK);
    t+=ARM_CONTROL_SETTLE_MS;
    ArmControl_Update(&c,&in,t);
    assert(c.state==ARM_BOOT_RESTORE);
    ArmControl_Update(&c,&in,t);
    assert(c.state==ARM_BOOT_NEED_REFERENCE);

    /* 位姿未知之前，Z 不产生任何作用。 */
    in.key=ARM_KEY_Z;
    ArmControl_Update(&c,&in,t);
    assert(c.mode==ARM_MODE_POWERLESS&&!c.custom_latched);
    in.key=0;
    ArmControl_Update(&c,&in,t);

    /* 兼容的记录只武装状态机，绝不放开发送。 */
    in.restore_ok=true; in.restore_known=true;
    ArmControl_Update(&c,&in,t);
    assert(c.state==ARM_BOOT_READY_DISARMED);
    assert(c.mode==ARM_MODE_POWERLESS&&!ArmControl_MotionAllowed(&c));

    /* Z 的上升沿只锁存一次自定义控制。 */
    in.key=ARM_KEY_Z;
    ArmControl_Update(&c,&in,t);
    assert(c.mode==ARM_MODE_CUSTOM&&c.custom_latched);
    assert(ArmControl_MotionAllowed(&c));
    t+=10; in.key=0;
    ArmControl_Update(&c,&in,t);
    assert(c.mode==ARM_MODE_CUSTOM);

    /* 对齐组合优先于锁存，并且只请求一次。 */
    in.s[0]=SW_DOWN; in.s[1]=SW_MID;
    t+=10; ArmControl_Update(&c,&in,t);
    assert(c.mode==ARM_MODE_ALIGN&&c.align_pending&&!c.custom_latched);
    assert(!ArmControl_MotionAllowed(&c));
    ArmControl_AlignDone(&c,true,t);
    assert(!c.align_pending);
    t+=10; ArmControl_Update(&c,&in,t);
    assert(c.mode==ARM_MODE_ALIGN&&!c.align_pending);

    /* 无人提交的对齐请求会过期，而不是反复重试。 */
    in.s[0]=SW_MID; in.s[1]=SW_MID;
    t+=10; ArmControl_Update(&c,&in,t);
    assert(c.mode==ARM_MODE_POWERLESS&&c.wait_idle);
    ArmControl_Update(&c,&in,t);
    assert(!c.wait_idle&&c.mode==ARM_MODE_POWERLESS);
    in.s[0]=SW_DOWN; in.s[1]=SW_MID;
    t+=10; ArmControl_Update(&c,&in,t);
    assert(c.align_pending);
    t+=ARM_CONTROL_ALIGN_TIMEOUT_MS+1;
    ArmControl_Update(&c,&in,t);
    assert(!c.align_pending&&c.mode==ARM_MODE_ALIGN);
    in.s[0]=SW_MID; in.s[1]=SW_MID;
    t+=10; ArmControl_Update(&c,&in,t);
    t+=10; ArmControl_Update(&c,&in,t);

    /* 双下停止手动与自定义控制，并清除锁存。 */
    in.key=ARM_KEY_Z;
    t+=10; ArmControl_Update(&c,&in,t);
    assert(c.mode==ARM_MODE_CUSTOM);
    in.s[0]=SW_DOWN; in.s[1]=SW_DOWN;
    t+=10; ArmControl_Update(&c,&in,t);
    assert(c.mode==ARM_MODE_POWERLESS&&!c.custom_latched);

    /* 进入手动模式仍需位姿就绪和有效的拨杆帧。 */
    in.s[0]=SW_DOWN; in.s[1]=SW_UP;
    t+=10; ArmControl_Update(&c,&in,t);
    assert(c.mode==ARM_MODE_MANUAL_J1_J4&&ArmControl_MotionAllowed(&c));
    in.online=false;
    t+=10; ArmControl_Update(&c,&in,t);
    assert(c.mode==ARM_MODE_POWERLESS&&!ArmControl_MotionAllowed(&c));
    in.online=true;

    /* 丢失来源会清掉锁存和尚未提交的对齐。 */
    in.key=ARM_KEY_Z; in.s[0]=SW_MID; in.s[1]=SW_MID;
    t+=10; ArmControl_Update(&c,&in,t);
    assert(c.mode==ARM_MODE_CUSTOM);
    in.online=false;
    t+=10; ArmControl_Update(&c,&in,t);
    assert(c.mode==ARM_MODE_POWERLESS&&!c.custom_latched&&!c.align_pending);
    in.online=true;

    /* 上电状态不算边沿，即使 Z 与组合都已按住。 */
    ArmControl_Reset(&c,t);
    in=input(SW_DOWN,SW_MID,ARM_KEY_Z);
    in.restore_ok=true; in.restore_known=true;
    ArmControl_Update(&c,&in,t);
    assert(c.mode==ARM_MODE_ALIGN&&!c.align_pending&&!c.custom_latched);

    /* 故障锁存后一直无力，直到复位。 */
    in.fault=true;
    t+=10; ArmControl_Update(&c,&in,t);
    assert(c.state==ARM_BOOT_FAULT&&c.mode==ARM_MODE_POWERLESS);
    in.fault=false;
    t+=10; ArmControl_Update(&c,&in,t);
    assert(c.state==ARM_BOOT_FAULT&&!ArmControl_MotionAllowed(&c));

    /* offset 默认 0；这里不编造周期，也不编造基准。 */
    ArmControl_Reset(&c,t);
    assert(ArmControl_AxisOffset(&c,0)==0.0f&&ArmControl_AxisOffset(&c,7)==0.0f);
    c.offset[3]=6.2831853f;
    assert(ArmControl_AxisOffset(&c,3)==6.2831853f);

    /* 源切换会重新建立边沿，并连同请求一起丢弃。 */
    ArmControl_Reset(&c,t);
    in=input(SW_MID,SW_MID,0);
    in.source=1; in.keyboard_source=1; in.restore_ok=true; in.restore_known=true;
    ArmControl_Update(&c,&in,t);
    t+=ARM_CONTROL_SETTLE_MS;
    ArmControl_Update(&c,&in,t);
    ArmControl_Update(&c,&in,t);
    assert(c.state==ARM_BOOT_READY_DISARMED);
    in.key=ARM_KEY_Z;
    t+=10; ArmControl_Update(&c,&in,t);
    assert(c.mode==ARM_MODE_CUSTOM&&c.custom_latched);
    in.source=2; in.keyboard_source=2;
    t+=10; ArmControl_Update(&c,&in,t);
    assert(c.mode==ARM_MODE_POWERLESS&&!c.custom_latched);
    t+=10; ArmControl_Update(&c,&in,t);
    assert(c.mode==ARM_MODE_POWERLESS);
}

static void restore_tests(void)
{
    float saved[7],raw[7],period[7],offset[7];
    ArmControl c; ArmControlInput in; uint32_t t=5000; unsigned i;

    /* 没在真实关节上实测过的周期，永远算不出 offset。 */
    for(i=0;i<7;i++){period[i]=0.0f;raw[i]=1.0f;saved[i]=1.0f;}
    assert(!ArmControl_SolveOffsets(saved,raw,period,offset));

    /* 位移小于半个周期：不绕圈，也不编造 offset。 */
    for(i=0;i<7;i++){period[i]=6.2831853f;raw[i]=0.5f;saved[i]=0.6f;}
    assert(ArmControl_SolveOffsets(saved,raw,period,offset));
    for(i=0;i<7;i++) assert(fabsf(offset[i])<1e-6f);

    /* 一整圈再加一个小位移，恢复出正好一个周期。 */
    for(i=0;i<7;i++) saved[i]=raw[i]+6.2831853f+0.1f;
    assert(ArmControl_SolveOffsets(saved,raw,period,offset));
    for(i=0;i<7;i++) assert(fabsf(offset[i]-6.2831853f)<1e-4f);

    /* 恰好半个周期存在歧义，保持未解决。 */
    for(i=0;i<7;i++) saved[i]=raw[i]+3.14159265f;
    assert(!ArmControl_SolveOffsets(saved,raw,period,offset));

    /* 非有限样本永远不能用于恢复。 */
    for(i=0;i<7;i++) saved[i]=raw[i]+0.1f;
    saved[2]=(float)NAN;
    assert(!ArmControl_SolveOffsets(saved,raw,period,offset));

    /* 启动会等待存储给出确定结论，但不会无限等下去。 */
    ArmControl_Reset(&c,t);
    in=input(SW_MID,SW_MID,0);
    in.restore_known=false;
    ArmControl_Update(&c,&in,t);
    t+=ARM_CONTROL_SETTLE_MS;
    ArmControl_Update(&c,&in,t);
    assert(c.state==ARM_BOOT_RESTORE);
    ArmControl_Update(&c,&in,t);
    assert(c.state==ARM_BOOT_RESTORE);
    t+=ARM_CONTROL_RESTORE_TIMEOUT_MS+1;
    ArmControl_Update(&c,&in,t);
    assert(c.state==ARM_BOOT_NEED_REFERENCE);
}

int main(void)
{
    storage_tests();gate_tests();parser_tests();control_tests();restore_tests();
    puts("PASS: log layout/CRC/288 power cuts/rollover/ECC-read failure; seven-axis gate; image stream; mode arbitration; restore math.");
    return 0;
}
