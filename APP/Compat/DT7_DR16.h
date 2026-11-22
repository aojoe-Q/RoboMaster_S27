#ifndef __DT7_DR16_H
#define __DT7_DR16_H

/*
 * The S27_TEST_1 template already receives the DT7/SBUS remote on UART5 and
 * stores it in remote_ctrl. The APP only uses the two switches from this
 * structure, so map the old DT7 name onto the template's remote_ctrl instead of
 * starting a second UART/DMA receiver.
 */

#include "Remote_Control.h"
#include "struct_typedef.h"

typedef Remote_Info_Typedef DT7_RC_ctrl_t;

#define DT7_rc_ctrl remote_ctrl

/* remote_ctrl is decoded with the same convention as Rcdata_t:
 * [0] = right switch, [1] = left switch. */
#define DT7_RIGHT_SW (remote_ctrl.rc.s[0])
#define DT7_LEFT_SW  (remote_ctrl.rc.s[1])

/* ------------------------------------------------------------------------- *
 * Remote switch / key definitions: single source of truth.
 * remote_ctrl and Rcdata_t both use [0] = right switch, [1] = left switch.
 * ------------------------------------------------------------------------- */
#ifndef RC_Left
#define RC_Left  1
#endif
#ifndef RC_Right
#define RC_Right 0
#endif

#ifndef RC_SW_UP
#define RC_SW_UP   ((uint16_t)1)
#endif
#ifndef RC_SW_MID
#define RC_SW_MID  ((uint16_t)3)
#endif
#ifndef RC_SW_DOWN
#define RC_SW_DOWN ((uint16_t)2)
#endif
#ifndef switch_is_up
#define switch_is_up(s)   ((s) == RC_SW_UP)
#endif
#ifndef switch_is_mid
#define switch_is_mid(s)  ((s) == RC_SW_MID)
#endif
#ifndef switch_is_down
#define switch_is_down(s) ((s) == RC_SW_DOWN)
#endif

/* Keyboard bits (only used by commented-out APP code, kept for build). */
#define KEY_PRESSED_OFFSET_W       ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S       ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A       ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D       ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT   ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL    ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q       ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E       ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R       ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F       ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G       ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z       ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X       ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C       ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V       ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B       ((uint16_t)1 << 15)

#endif /* __DT7_DR16_H */
