/**
  ******************************************************************************
  * @file    Arm_USB_task.h
  * @brief   USB CDC link between the arm and ROS2 / MoveIt2.
  *
  * Data flow:
  *   CDC_Init_HS / CDC_DeInit_HS (IRQ) -> Arm_USB_OnSessionStart / End
  *   CDC_Receive_HS (IRQ)             -> Arm_USB_RxPush -> RX ring
  *   Arm_USB_Task (100 Hz)            -> parse, validate, update the PC
  *                                       command buffer, send STATE/ACK/FAULT
  *   Arm_L_Task                       -> Arm_USB_Get_PC_Target()
  *
  * The USB task never writes ArmData. PC control starts disabled, needs an
  * ENABLE in the current USB session (refused until Arm_L_Init has finished
  * and every joint has fresh CAN feedback), and drops back to disabled on
  * link timeout, session end, USB suspend or STOP.
  ******************************************************************************
  */

#ifndef __ARM_USB_TASK_H
#define __ARM_USB_TASK_H

#include <stdint.h>
#include "arm_usb_protocol.h"

#define ARM_USB_TASK_PERIOD_MS      10U     /* STATE at 100 Hz                      */
#define ARM_USB_LINK_TIMEOUT_MS     300U    /* no valid frame -> disable PC control  */
#define ARM_USB_CMD_TIMEOUT_MS      300U    /* max life of one POSITION_CMD          */
#define ARM_USB_CMD_MIN_VALID_MS    10U     /* floor for the host valid_time_ms      */
#define ARM_USB_RX_STALL_MS         20U     /* partial frame older than this: resync */
#define ARM_USB_FEEDBACK_TIMEOUT_MS 100U    /* joint feedback older: not valid       */
#define ARM_USB_OFFSET_CREEP_MS     1000U   /* clock drift tracking, 1 ms per period */
#define ARM_USB_ACK_QUEUE_LEN       8U

typedef struct
{
    uint32_t tx_frames;         /* frames handed to CDC               */
    uint32_t tx_busy;           /* ticks skipped, CDC still sending   */
    uint32_t tx_ack_dropped;    /* ACK queue overflow                 */
    uint32_t rx_frames;         /* frames with valid CRC              */
    uint32_t rx_crc_errors;
    uint32_t rx_resync_bytes;   /* bytes skipped while hunting SOF    */
    uint32_t rx_rejected;       /* valid CRC but refused (see ACK)    */
    uint32_t rx_stale;          /* POSITION_CMD older than the last or
                                   queued past its valid time         */
    uint32_t rx_overflow_bytes; /* bytes the RX ring refused          */
} USB_Stats_t;

extern USB_Stats_t usb_stats;

/* FreeRTOS entry, created in Core/Src/freertos.c. */
void Arm_USB_Task(void const *argument);

/* Called once by the task; exposed for the host self-test. */
void Arm_USB_Init(void);

/* One task period: session check, RX parse, timeouts, TX. */
void Arm_USB_Step(uint32_t now_ms);

/* USB interrupt context. */
void Arm_USB_OnSessionStart(void);   /* CDC_Init_HS   */
void Arm_USB_OnSessionEnd(void);     /* CDC_DeInit_HS */
void Arm_USB_RxPush(const uint8_t *data, uint32_t len);

/* PC control enabled (ENABLE received in this session, no fault since). */
uint8_t Arm_USB_PC_Enabled(void);

/* Arm_L_Task left ARM_MODE_PC_CONTROL (RC switch moved): drop the enable and
   the command, so re-entering the mode needs a new ENABLE from the host.
   Task context. */
void Arm_USB_PC_Disarm(void);

/* Copy the PC joint targets (rad) when enabled, in the same USB session and
   the command is fresh. Returns 1 on success, 0 otherwise (targets
   untouched). Task context. */
uint8_t Arm_USB_Get_PC_Target(float target_rad[ARM_PROTO_JOINT_NUM]);

#endif /* __ARM_USB_TASK_H */
