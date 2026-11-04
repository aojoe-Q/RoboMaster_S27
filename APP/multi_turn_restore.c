/**
  ******************************************************************************
  * @file    multi_turn_restore.c
  * @brief   Boot offset b of the seven joints, restore checks and the save
  *          trigger of the power-loss log (see the header).
  ******************************************************************************
  */

#include "multi_turn_restore.h"
#include "multi_turn_store.h"
#include "Arm_L_task.h"
#include "referee_usart_task.h"
#include "can_bsp.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
#include <string.h>

float Joint_Boot_Offset[7];
volatile MT_Restore_State_t MT_Restore_State = MT_RESTORE_BOOT;

#define MT_BOOT_FEEDBACK_WAIT_MS    500U

static const float joint_max[7] = {
    J1_Max_position, J2_Max_position, J3_Max_position, J4_Max_position,
    J5_Max_position, J6_Max_position, Grip_Max_position
};
static const float joint_min[7] = {
    J1_Min_position, J2_Min_position, J3_Min_position, J4_Min_position,
    J5_Min_position, J6_Min_position, Grip_Min_position
};
static const float joint_zero[7] = {
    J1_zero_position, J2_zero_position, J3_zero_position, J4_zero_position,
    J5_zero_position, J6_zero_position, grip_zero_position
};

static float    saved_pos[7];
static uint8_t  saved_valid;
static uint32_t last_save_tick;
static uint32_t still_since_tick;
static uint8_t  still;

/* ---------------------------------------------------------- config id */

static uint32_t cfg_put_f32(uint8_t *buf, uint32_t n, float v)
{
    memcpy(&buf[n], &v, 4);
    return n + 4U;
}

uint32_t MT_Config_Id(void)
{
    uint8_t  buf[160];
    uint32_t n = 0;

    buf[n++] = (uint8_t)MT_FORMAT_VERSION;
    buf[n++] = (uint8_t)MT_RESTORE_ALGO_VERSION;
    buf[n++] = (uint8_t)MT_JOINT_NUM;
    for (uint8_t i = 0; i < 7U; i++)
    {
        buf[n++] = (uint8_t)(CAN_DM_L_J1_ID + i);   /* axis order = CAN ID 0x01..0x07 */
    }
    for (uint8_t i = 0; i < 7U; i++)
    {
        n = cfg_put_f32(buf, n, Joint_P_MIN[i]);
        n = cfg_put_f32(buf, n, Joint_P_MAX[i]);
    }
    n = cfg_put_f32(buf, n, MT_JOINT_PERIOD);
    for (uint8_t i = 0; i < 7U; i++)
    {
        n = cfg_put_f32(buf, n, joint_zero[i]);
        n = cfg_put_f32(buf, n, joint_min[i]);
        n = cfg_put_f32(buf, n, joint_max[i]);
    }
    return MT_Crc32(buf, n);
}

/* ------------------------------------------------------------ restore */

uint8_t MT_Restore_Compute(const float saved[7], const float raw[7], float b_out[7])
{
    float b[7];

    for (uint8_t i = 0; i < 7U; i++)
    {
        float turns, p_now;

        if (!isfinite(saved[i]) || !isfinite(raw[i]))
        {
            return 0U;
        }
        turns = roundf((saved[i] - raw[i]) / MT_JOINT_PERIOD);
        b[i]  = turns * MT_JOINT_PERIOD;
        p_now = raw[i] + b[i];
        {
            const float tol = (i == 6U) ? MT_RESTORE_RESIDUAL_GRIP : MT_RESTORE_RESIDUAL_MAX;
            if (fabsf(p_now - saved[i]) > tol)
            {
                return 0U;      /* moved too far since the record, or wrong T */
            }
        }
        /* joint_min/max 是 raw 标定下的物理限位，而 p_now 是逻辑坐标；
           b != 0 时必须换回物理位置再比，否则整条记录永远越界。 */
        if ((raw[i] > joint_max[i] + MT_RESTORE_LIMIT_MARGIN) ||
            (raw[i] < joint_min[i] - MT_RESTORE_LIMIT_MARGIN))
        {
            return 0U;
        }
    }
    memcpy(b_out, b, sizeof(b));
    return 1U;
}

void MT_Restore_Boot(void)
{
    uint32_t start = xTaskGetTickCount();
    MT_Record_t rec;
    float raw[7];
    float b[7];
    uint8_t mask;

    memset(Joint_Boot_Offset, 0, sizeof(Joint_Boot_Offset));
    saved_valid = 0U;

    do
    {
        taskENTER_CRITICAL();
        mask = APP_Joint_Fresh_Mask(xTaskGetTickCount());
        for (uint8_t i = 0; i < 7U; i++)
        {
            raw[i] = joint_L_motor[i].position;
        }
        taskEXIT_CRITICAL();
        if (mask == JOINT_ALL_MASK)
        {
            break;
        }
        osDelay(5);
    } while ((xTaskGetTickCount() - start) < MT_BOOT_FEEDBACK_WAIT_MS);

    if (MT_Store_Get_Latest(&rec) && (rec.config_id == MT_Config_Id()) &&
        (rec.flags & MT_FLAG_SLAVE_POS_VALID) && ((rec.slave_valid_mask & 0x7FU) == 0x7FU))
    {
        memcpy(saved_pos, rec.slave_logical_pos, sizeof(saved_pos));
        saved_valid = 1U;
    }

#if MT_RESTORE_ENABLE
    if (saved_valid && (mask == JOINT_ALL_MASK) && MT_Restore_Compute(saved_pos, raw, b))
    {
        memcpy(Joint_Boot_Offset, b, sizeof(b));
        MT_Restore_State = MT_RESTORE_READY;
    }
    else
    {
        /* No fallback to b = 0 with the old limits: the logical position
           is unknown. */
        MT_Restore_State = MT_RESTORE_NEED_REFERENCE;
    }
#else
    (void)b;
    (void)raw;
    MT_Restore_State = MT_RESTORE_DISABLED;
#endif
    last_save_tick = xTaskGetTickCount();
}

uint8_t MT_Restore_Output_Allowed(void)
{
    return (uint8_t)((MT_Restore_State == MT_RESTORE_DISABLED) ||
                     (MT_Restore_State == MT_RESTORE_READY));
}

/* --------------------------------------------------------------- save */

static void save_record(const float pos[7])
{
    MT_Record_t rec;
    uint32_t tick = 0;

    memset(&rec, 0, sizeof(rec));
    rec.config_id        = MT_Config_Id();
    rec.flags            = MT_FLAG_SLAVE_POS_VALID;
    rec.slave_valid_mask = 0x7FU;
    rec.calibration_state = (MT_Restore_State == MT_RESTORE_READY) ? MT_CAL_RESTORED : MT_CAL_LEGACY;
#if MT_RESTORE_ENABLE
    rec.flags |= MT_FLAG_RESTORE_ENABLED;
#endif
    memcpy(rec.slave_logical_pos, pos, sizeof(rec.slave_logical_pos));

    /* Master: protocol not confirmed. Raw payload for diagnostics only;
       decoded fields and masks stay invalid. */
    taskENTER_CRITICAL();
    if (Master_Raw.rx_count != 0U)
    {
        memcpy(rec.master_payload, Master_Raw.data, MT_MASTER_PAYLOAD_LEN);
        rec.flags |= MT_FLAG_MASTER_PAYLOAD;
        tick = Master_Raw.rx_tick;
    }
    taskEXIT_CRITICAL();
    (void)tick;

    if (MT_Store_Append(&rec))
    {
        memcpy(saved_pos, pos, sizeof(saved_pos));
        saved_valid = 1U;
    }
    last_save_tick = xTaskGetTickCount();
}

void MT_Restore_Process(uint8_t save_window)
{
    uint32_t now = xTaskGetTickCount();
    float pos[7];
    float vel_max = 0.0f;
    float delta_max = 0.0f;
    uint8_t mask;

    taskENTER_CRITICAL();
    mask = APP_Joint_Fresh_Mask(now);
    for (uint8_t i = 0; i < 7U; i++)
    {
        float v = fabsf(joint_L_motor[i].velocity);
        pos[i] = JOINT_LOGICAL_POSITION(i);
        if (v > vel_max)
        {
            vel_max = v;
        }
    }
    taskEXIT_CRITICAL();

#if MT_RESTORE_ENABLE
    /* A joint that stops feeding back may have rebooted: its raw p can jump
       by one period, so b is no longer known. Latched until reset. */
    if ((MT_Restore_State == MT_RESTORE_READY) && (mask != JOINT_ALL_MASK))
    {
        MT_Restore_State = MT_RESTORE_FAULT;
    }
#endif

    if (!MT_Restore_Output_Allowed() || !save_window || (mask != JOINT_ALL_MASK) ||
        (MT_Store_Status.state != MT_STORE_OK) || !(vel_max < MT_SAVE_STILL_VEL))
    {
        still = 0U;
        return;
    }
    if (!still)
    {
        still = 1U;
        still_since_tick = now;
        return;
    }
    if ((now - still_since_tick) < MT_SAVE_STILL_MS ||
        (now - last_save_tick) < MT_SAVE_MIN_INTERVAL_MS)
    {
        return;
    }
    for (uint8_t i = 0; i < 7U; i++)
    {
        float d = fabsf(pos[i] - saved_pos[i]);
        if (!isfinite(pos[i]))
        {
            return;
        }
        if (d > delta_max)
        {
            delta_max = d;
        }
    }
    if (saved_valid && (delta_max < MT_SAVE_MIN_DELTA))
    {
        return;
    }
    save_record(pos);
}
