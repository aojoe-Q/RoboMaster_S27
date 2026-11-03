#include <math.h>
#include "arm_control.h"

#define SWITCH_UP   1u
#define SWITCH_DOWN 2u
#define SWITCH_MID  3u

static bool switch_valid(uint8_t s)
{
    return s >= 1u && s <= 3u;
}

static void boot_advance(ArmControl *c, const ArmControlInput *in, uint32_t now)
{
    if (in->fault) {
        if (c->state != ARM_BOOT_FAULT) {
            c->state = ARM_BOOT_FAULT;
            c->state_tick = now;
        }
        return;
    }

    switch (c->state) {
    case ARM_BOOT_FAULT:
        return;
    case ARM_BOOT_LOCKED:
        if (in->axes_ready) {
            c->state = ARM_BOOT_WAIT_FEEDBACK;
            c->state_tick = now;
        }
        break;
    case ARM_BOOT_WAIT_FEEDBACK:
        if (!in->axes_ready) {
            c->state = ARM_BOOT_LOCKED;
            c->state_tick = now;
        } else if ((uint32_t)(now - c->state_tick) >= ARM_CONTROL_SETTLE_MS) {
            c->state = ARM_BOOT_RESTORE;
            c->state_tick = now;
        }
        break;
    case ARM_BOOT_RESTORE:
        if (!in->axes_ready) {
            c->state = ARM_BOOT_WAIT_FEEDBACK;
            c->state_tick = now;
        } else if (!in->restore_known &&
                   (uint32_t)(now - c->state_tick) <= ARM_CONTROL_RESTORE_TIMEOUT_MS) {
            break; /* 等待存储给出结论，最多等到上面的超时。 */
        } else if (in->restore_ok) {
            c->state = ARM_BOOT_READY_DISARMED;
            c->state_tick = now;
        } else {
            c->state = ARM_BOOT_NEED_REFERENCE;
            c->state_tick = now;
        }
        break;
    default:
        if (!in->axes_ready) {
            c->state = ARM_BOOT_LOCKED;
            c->state_tick = now;
        } else if (in->restore_ok && c->state != ARM_BOOT_READY_DISARMED) {
            c->state = ARM_BOOT_READY_DISARMED;
            c->state_tick = now;
        }
        break;
    }
}

/* 复位仲裁状态：启动重新上锁，模式回到无力，清掉全部锁存与边沿。 */
void ArmControl_Reset(ArmControl *c, uint32_t now)
{
    unsigned i;

    c->state = ARM_BOOT_LOCKED;
    c->mode = ARM_MODE_POWERLESS;
    c->custom_latched = false;
    c->align_pending = false;
    c->align_active = false;
    c->wait_idle = false;
    c->primed = false;
    c->z_prev = false;
    c->combo_prev = false;
    c->state_tick = now;
    c->align_tick = now;
    c->source = 0;
    c->keyboard_source = 0;
    for (i = 0; i < ARM_CONTROL_AXES; i++) {
        c->offset[i] = 0.0f;
    }
}

/* 每个控制周期调用一次：推进启动状态机并仲裁模式。 */
void ArmControl_Update(ArmControl *c, const ArmControlInput *in, uint32_t now)
{
    bool switches_ok;
    bool combo;
    bool z;
    bool z_edge;
    bool combo_edge;

    boot_advance(c, in, now);

    switches_ok = in->online && switch_valid(in->s[0]) && switch_valid(in->s[1]);
    combo = switches_ok && in->s[1] == SWITCH_MID && in->s[0] == SWITCH_DOWN;
    z = in->online && (in->key & ARM_KEY_Z) != 0u;

    /* 输入源切换后要重新建立边沿：新源上已经按住的键不算一次新的按下，
       上电时的初始状态同样不算边沿。 */

    if (c->source != in->source || c->keyboard_source != in->keyboard_source) {
        c->source = in->source;
        c->keyboard_source = in->keyboard_source;
        c->primed = false;
        /* 不同发射机发起的请求不通用：丢弃锁存和尚未提交的对齐。 */

        c->custom_latched = false;
        c->align_pending = false;
    }
    if (!c->primed) {
        c->primed = true;
        c->z_prev = z;
        c->combo_prev = combo;
    }
    z_edge = z && !c->z_prev;
    combo_edge = combo && !c->combo_prev;
    c->z_prev = z;
    c->combo_prev = combo;

    /* 1. 明确的停止：撤出力、撤销待执行对齐、清 Z 锁存。 */
    if (in->fault || !switches_ok) {
        c->align_pending = false;
        c->align_active = false;
        c->custom_latched = false;
        c->wait_idle = false;
        c->mode = ARM_MODE_POWERLESS;
        return;
    }

    /* 2. 对齐组合优先于其它一切请求。 */
    if (combo) {
        if (combo_edge) {
            c->custom_latched = false;
            c->align_pending = true;
            c->align_tick = now;
        }
        if (c->align_pending &&
            (uint32_t)(now - c->align_tick) > ARM_CONTROL_ALIGN_TIMEOUT_MS) {
            c->align_pending = false; /* 只取消一次，按住期间不重试 */
        }
        c->align_active = true;
        c->mode = ARM_MODE_ALIGN;
        return;
    }

    /* 退出组合：在双中位之前一直保持无力。 */
    if (c->align_active) {
        c->align_active = false;
        c->align_pending = false;
        c->wait_idle = true;
        c->mode = ARM_MODE_POWERLESS;
        return;
    }
    if (c->wait_idle) {
        c->mode = ARM_MODE_POWERLESS;
        if (in->s[0] == SWITCH_MID && in->s[1] == SWITCH_MID) {
            c->wait_idle = false;
        }
        return;
    }

    /* 3. 双下是手动与自定义控制的明确停止。 */
    if (in->s[0] == SWITCH_DOWN && in->s[1] == SWITCH_DOWN) {
        c->custom_latched = false;
        c->align_pending = false;
        c->mode = ARM_MODE_POWERLESS;
        return;
    }

    /* 4. 新请求：位姿已知时，Z 键锁存自定义控制。 */
    if (z_edge && c->state == ARM_BOOT_READY_DISARMED) {
        c->custom_latched = true;
    }

    /* 5. 已选中且仍然有效的模式保持优先。 */
    if (c->custom_latched) {
        c->mode = ARM_MODE_CUSTOM;
        return;
    }
    if (in->s[1] == SWITCH_UP && in->s[0] == SWITCH_DOWN) {
        c->mode = ARM_MODE_MANUAL_J1_J4;
        return;
    }
    if (in->s[0] == SWITCH_UP && in->s[1] == SWITCH_UP) {
        c->mode = ARM_MODE_TAKE_WATER;
        return;
    }

    c->mode = ARM_MODE_POWERLESS;
}

/* 只有位姿已知、且当前模式确实要求出力时才允许输出。 */
bool ArmControl_MotionAllowed(const ArmControl *c)
{
    if (c->state != ARM_BOOT_READY_DISARMED) {
        return false;
    }
    switch (c->mode) {
    case ARM_MODE_MANUAL_J1_J4:
    case ARM_MODE_TAKE_WATER:
    case ARM_MODE_CUSTOM:
        return true;
    default:
        return false;
    }
}

/* 读取某轴的 boot_offset_rad；轴号非法或尚未定义时返回 0。 */
float ArmControl_AxisOffset(const ArmControl *c, unsigned axis)
{
    if (axis >= ARM_CONTROL_AXES) {
        return 0.0f;
    }
    return c->offset[axis];
}

/* 按保存的逻辑位姿和上电原始反馈反解 boot_offset_rad（方案 3.2 节）。 */
bool ArmControl_SolveOffsets(const float saved[ARM_CONTROL_AXES],
                             const float raw[ARM_CONTROL_AXES],
                             const float period[ARM_CONTROL_AXES],
                             float offset[ARM_CONTROL_AXES])
{
    float b[ARM_CONTROL_AXES];
    unsigned i;

    for (i = 0; i < ARM_CONTROL_AXES; i++) {
        float t = period[i];
        float n, residual;

        if (!(t > 0.0f) || !isfinite(t) || !isfinite(saved[i]) || !isfinite(raw[i])) {
            return false;
        }
        n = roundf((saved[i] - raw[i]) / t);
        b[i] = n * t;
        residual = (raw[i] + b[i]) - saved[i];
        if (!isfinite(residual) || fabsf(residual) >= 0.5f * t) {
            return false;
        }
    }
    for (i = 0; i < ARM_CONTROL_AXES; i++) {
        offset[i] = b[i];
    }
    return true;
}

/* 锁存一次对齐请求：先等无力输出确认，再整组提交，不重复触发。 */
void ArmControl_RequestAlign(ArmControl *c, uint32_t now)
{
    c->align_pending = true;
    c->align_tick = now;
}

/* 对齐结束（成功或失败）；失败转入 NEED_MASTER_ALIGN，不重试。 */
void ArmControl_AlignDone(ArmControl *c, bool ok, uint32_t now)
{
    c->align_pending = false;
    if (!ok && c->state == ARM_BOOT_READY_DISARMED) {
        c->state = ARM_BOOT_NEED_MASTER_ALIGN;
    }
    c->state_tick = now;
}

/* 启动状态的可读名称，供调试器查看。 */
const char *ArmControl_StateName(ArmBootState state)
{
    switch (state) {
    case ARM_BOOT_LOCKED:     return "BOOT_LOCKED";
    case ARM_BOOT_WAIT_FEEDBACK: return "WAIT_MOTOR_FEEDBACK";
    case ARM_BOOT_RESTORE:    return "RESTORE_SLAVES";
    case ARM_BOOT_READY_DISARMED:  return "READY_DISARMED";
    case ARM_BOOT_NEED_REFERENCE:  return "NEED_REFERENCE";
    case ARM_BOOT_NEED_MASTER_ALIGN: return "NEED_MASTER_ALIGN";
    case ARM_BOOT_FAULT:           return "FAULT";
    default:                  return "UNKNOWN";
    }
}
