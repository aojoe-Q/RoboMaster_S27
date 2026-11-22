#ifndef VT13_REMOTE_CONTROL_H
#define VT13_REMOTE_CONTROL_H

#include <stdint.h>

#pragma pack(push, 1)
typedef struct
{
    struct
    {
        int16_t ch[4];
        uint8_t mode_sw;
        uint8_t stop;
        uint8_t left_button;
        uint8_t right_button;
        int16_t wheel;
        uint8_t shutter;
    } rc;
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
        uint8_t middle;
    } mouse;
    struct
    {
        uint16_t v;
    } key;
    uint16_t crc16;
} VT13_RC_ctrl_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
    uint32_t count;
    uint8_t  last_state;
    uint8_t  debounce_cnt;
    uint8_t  stable_state;
} ButtonCounter;
#pragma pack(pop)

void button_init(ButtonCounter *btn);
void button_update(ButtonCounter *btn, uint8_t current_raw_state);
uint32_t button_get_count(ButtonCounter *btn);
void button_reset(ButtonCounter *btn);

#endif /* VT13_REMOTE_CONTROL_H */
