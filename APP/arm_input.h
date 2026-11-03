#ifndef ARM_INPUT_H
#define ARM_INPUT_H
#include <stdint.h>
#include <stdbool.h>
typedef struct {
    uint32_t source, switches_tick, keyboard_tick, seq;
    uint8_t s[2];
    uint16_t key;
    /* 键盘来源：1 = DT7/VT13，2 = 影子板，3 = 图传 0x0304。
       两路键盘按位或合并是禁止的。 */
    uint8_t keyboard_source;
    bool online;
} ArmInputSnapshot;
void ArmInput_Read(ArmInputSnapshot *out);
extern volatile uint32_t arm_dt7_tick,arm_dt7_seq;
#endif
