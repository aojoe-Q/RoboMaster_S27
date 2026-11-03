#include "arm_gate.h"
#include <math.h>
#include <string.h>
static bool in(float x,float lo,float hi) { return isfinite(x)&&x>=lo&&x<=hi; }
/* 撤销输出许可：下一次下发必然被拒，并推进命令代次。 */
void ArmGate_Revoke(ArmGate *g) { g->permitted=false; g->epoch++; }
/* 整臂校验后取出一轴命令；任一项不满足即撤销许可并退回原始反馈位置。 */
bool ArmGate_Select(ArmGate *g,unsigned axis,uint32_t now,ArmAxisCommand *out)
{
    unsigned i;
    memset(out,0,sizeof(*out));
    if (axis>=ARM_AXES) return false;
    if (!g->permitted || !g->configured || g->maintenance || !g->input_valid ||
        g->command.epoch!=g->epoch || g->command.source!=g->source ||
        (uint32_t)(now-g->input_tick)>100 || (uint32_t)(now-g->command.tick)>50)
        goto reject;
    /* 每次下发都做整臂校验，包括夹爪力矩。 */
    for(i=0;i<ARM_AXES;i++) {
        const ArmAxisFeedback *f=&g->feedback[i];
        const ArmAxisCommand *c=&g->command.axis[i];
        if (!f->valid || !f->referenced || f->state!=1 ||
            (uint32_t)(now-f->tick)>100 ||
            !in(f->raw,-12.5f,12.5f) || !isfinite(f->offset) ||
            !isfinite(f->minimum) || !isfinite(f->maximum) || f->minimum>f->maximum ||
            !in(f->raw+f->offset,f->minimum,f->maximum) ||
            !in(c->position,f->minimum,f->maximum) ||
            !in(c->position-f->offset,-12.5f,12.5f) ||
            !in(c->velocity,-45,45) || !in(c->kp,0,500) ||
            !in(c->kd,0,5) || !in(c->torque,-54,54)) goto reject;
    }
    *out=g->command.axis[axis];
    out->position-=g->feedback[axis].offset;
    return true;
reject:
    if (g->permitted) ArmGate_Revoke(g);
    if (in(g->feedback[axis].raw,-12.5f,12.5f)) out->position=g->feedback[axis].raw;
    return false;
}
