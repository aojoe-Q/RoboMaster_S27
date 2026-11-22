#include <assert.h>
#include <stdio.h>
#include "arm_motion.h"
#include "arm_output.h"

static ArmData_t ArmData;
static DM_contorl_t *const arm_joint_control[] = {
    &ArmData.J1, &ArmData.J2, &ArmData.J3, &ArmData.J4, &ArmData.J5, &ArmData.J6, &ArmData.Grip
};
static ramp_function_source_t J1_L_position_set, J2_L_position_set, J3_L_position_set,
    J4_L_position_set, J5_L_position_set, J6_L_position_set, grip_L_position_set;
static uint8_t Arm_L_Ready = 1, grip_torque_mode = 1, valid_target;
static float s_grip_blend = 1.0f;
static Arm_Control_Snapshot arm_pending_command, published;
ArmOutputDiagnostics arm_output_diag;
static uint32_t now;
#define portTICK_PERIOD_MS 1U
static uint32_t xTaskGetTickCount(void) { return now; }
void Arm_Control_Publish(const Arm_Control_Snapshot *cmd) { published = *cmd; }
static uint8_t Arm_USB_Get_PC_Command(float *target, float *vmax)
{
    if (!valid_target) return 0;
    for (unsigned joint = 0; joint < 7; ++joint) {
        target[joint] = arm_joint_control[joint]->position + 0.1f;
        vmax[joint] = 0.5f;
    }
    return 1;
}

/* REAL_FUNCTIONS */

static void step(void)
{
    now += 8;
    Arm_PC_Control();
    Arm_PrepareAndPublish();
    assert(published.tick_ms == now);
}

static void check_read_only(void)
{
    for (unsigned joint = 0; joint < 7; ++joint) {
        assert(published.kp[joint] == 0 && published.kd[joint] == 0 && published.torque[joint] == 0);
        assert(published.pos[joint] == arm_joint_control[joint]->position);
    }
}

int main(void)
{
    ArmData.ArmMode = ARM_MODE_PC_CONTROL;
    const float initial[] = {0, -1, -1, 1, 0, 0, -1};
    for (unsigned joint = 0; joint < 7; ++joint) {
        arm_joint_control[joint]->position = initial[joint];
        arm_joint_control[joint]->Kp = arm_joint_control[joint]->Kd = arm_joint_control[joint]->torque = 1;
    }
    Arm_PC_Control_Enter();
    step(); check_read_only();
    ArmData.J1.position = 0.5f;
    step(); check_read_only();
    valid_target = 1;
    step();
    for (unsigned joint = 0; joint < 7; ++joint) {
        float travel = published.pos[joint] - arm_joint_control[joint]->position;
        assert(published.kp[joint] > 0 && published.torque[joint] == 0);
        assert(travel > 0 && travel < 0.001f);
    }
    valid_target = 0;
    step();
    ArmData.J1.position += 0.05f;
    step();
    assert(published.kp[0] > 0 && published.pos[0] == 0.5f);
    Arm_PC_Control_Enter();
    step(); check_read_only();
    puts("PC mode: read-only, takeover, hold and re-entry OK");
    return 0;
}
