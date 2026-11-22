#ifndef CONTROL_HAL_H
#define CONTROL_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef uint32_t TickType_t;
typedef int BaseType_t;
typedef void *TaskHandle_t;
#define pdFALSE 0
#define pdTRUE 1
#define portTICK_PERIOD_MS 1U
#define pdMS_TO_TICKS(x) (x)
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define portYIELD_FROM_ISR(x) ((void)(x))
#define __DMB() __asm__ volatile("" ::: "memory")
#define CLEAR_BIT(reg,mask) ((reg) &= ~(mask))
#define APP_ARM_ENABLE 1
typedef struct { uint32_t PSR, CCCR, TXBRP; } Registers;
typedef struct { Registers *Instance; } FDCAN_HandleTypeDef;
typedef FDCAN_HandleTypeDef hcan_t;
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
typedef struct {
    uint32_t Identifier, IdType, TxFrameType, DataLength, ErrorStateIndicator;
    uint32_t BitRateSwitch, FDFormat, TxEventFifoControl, MessageMarker;
} FDCAN_TxHeaderTypeDef;
typedef struct { uint32_t Identifier, IdType, RxFrameType, DataLength; } FDCAN_RxHeaderTypeDef;
typedef struct { uint32_t IdType, FilterIndex, FilterType, FilterConfig, FilterID1, FilterID2; } FDCAN_FilterTypeDef;
typedef struct { uint32_t TxErrorCnt, RxErrorCnt; } FDCAN_ErrorCountersTypeDef;
typedef struct {
    uint32_t motorid;
    uint8_t status;
    float position, velocity, torque;
    uint32_t update_tick, update_count;
} joint_motor_measure_t;
extern joint_motor_measure_t joint_L_motor[7];
extern FDCAN_HandleTypeDef hfdcan1,hfdcan2,hfdcan3;
#define FDCAN_STANDARD_ID 0U
#define FDCAN_DATA_FRAME 0U
#define FDCAN_DLC_BYTES_8 8U
#define FDCAN_ESI_ACTIVE 0U
#define FDCAN_BRS_OFF 0U
#define FDCAN_BRS_ON 1U
#define FDCAN_CLASSIC_CAN 0U
#define FDCAN_FD_CAN 1U
#define FDCAN_NO_TX_EVENTS 0U
#define FDCAN_FILTER_MASK 0U
#define FDCAN_FILTER_TO_RXFIFO0 0U
#define FDCAN_FILTER_TO_RXFIFO1 1U
#define FDCAN_REJECT 0U
#define FDCAN_REJECT_REMOTE 0U
#define FDCAN_IT_BUS_OFF 1U
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 2U
#define FDCAN_IT_RX_FIFO1_NEW_MESSAGE 4U
#define FDCAN_IT_RX_FIFO0_MESSAGE_LOST 8U
#define FDCAN_IT_RX_FIFO1_MESSAGE_LOST 16U
#define FDCAN_RX_FIFO0 0U
#define FDCAN_RX_FIFO1 1U
#define FDCAN_PSR_BO 1U
#define FDCAN_PSR_EP 2U
#define FDCAN_CCCR_INIT 1U
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define T_MIN -10.0f
#define T_MAX 10.0f
uint32_t xTaskGetTickCount(void);
uint32_t xTaskGetTickCountFromISR(void);
void vTaskDelayUntil(TickType_t *,TickType_t);
void osDelay(uint32_t);
TaskHandle_t xTaskGetCurrentTaskHandle(void);
void vTaskNotifyGiveFromISR(TaskHandle_t,BaseType_t *);
void Error_Handler(void);
float uint_to_float(int,float,float,int);
HAL_StatusTypeDef canx_send_data(FDCAN_HandleTypeDef *,uint16_t,uint8_t *,uint32_t);
void APP_CAN_RxHandler(FDCAN_HandleTypeDef *,uint32_t,uint8_t *,uint32_t);
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *,FDCAN_TxHeaderTypeDef *,uint8_t *);
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *,FDCAN_FilterTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *,uint32_t,uint32_t,uint32_t,uint32_t);
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *,uint32_t,uint32_t);
HAL_StatusTypeDef HAL_FDCAN_ConfigTxDelayCompensation(FDCAN_HandleTypeDef *,uint32_t,uint32_t);
HAL_StatusTypeDef HAL_FDCAN_EnableTxDelayCompensation(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *);
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *,uint32_t);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *,uint32_t,FDCAN_RxHeaderTypeDef *,uint8_t *);
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef *,FDCAN_ErrorCountersTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *,uint32_t);
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *,uint32_t);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *,uint32_t);
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *,uint32_t);
#endif
