#ifndef ARM_GATE_H
#define ARM_GATE_H
#include <stdint.h>
#include <stdbool.h>
#define ARM_AXES 7
typedef struct { float position, velocity, kp, kd, torque; } ArmAxisCommand;
typedef struct {
    ArmAxisCommand axis[ARM_AXES];
    uint32_t epoch, tick, seq, source;
} ArmCommandFrame;
typedef struct {
    float raw, offset, minimum, maximum;
    uint32_t tick;
    uint8_t state;
    bool valid, referenced;
} ArmAxisFeedback;
typedef struct {
    ArmCommandFrame command;
    ArmAxisFeedback feedback[ARM_AXES];
    uint32_t epoch, source, input_tick;
    bool permitted, input_valid, maintenance, configured;
} ArmGate;
void ArmGate_Revoke(ArmGate *gate);
bool ArmGate_Select(ArmGate *gate, unsigned axis, uint32_t now, ArmAxisCommand *out);
#endif
