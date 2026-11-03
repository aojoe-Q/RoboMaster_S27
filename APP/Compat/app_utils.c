#include "VT13_rc_ctrl.h"

/*
 * Button debounce helpers copied from the S27_ARM_H7 VT13 driver.
 * The VT13 UART link itself is not needed by the arm task (the original code
 * has the VT13 control path commented out), so only these helpers are kept.
 */

void button_init(ButtonCounter *btn)
{
    btn->count        = 0;
    btn->last_state   = 0;
    btn->debounce_cnt = 0;
    btn->stable_state = 0;
}

void button_update(ButtonCounter *btn, uint8_t current_raw_state)
{
    if (current_raw_state == btn->stable_state)
    {
        btn->debounce_cnt = 0;
    }
    else
    {
        btn->debounce_cnt++;
        if (btn->debounce_cnt >= 3U)
        {
            btn->stable_state = current_raw_state;
            btn->debounce_cnt = 0;

            if ((btn->last_state == 0U) && (btn->stable_state == 1U))
            {
                btn->count++;
            }
            btn->last_state = btn->stable_state;
        }
    }
}

uint32_t button_get_count(ButtonCounter *btn)
{
    return btn->count;
}

void button_reset(ButtonCounter *btn)
{
    btn->count = 0;
}
