/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : MiniPC.c
  * @brief          : MiniPC interfaces functions
  * @author         : GarssFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "Minipc.h"
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceHS;

bool MiniPC_Is_Connected(void)
{
    return (hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED) &&
           (hUsbDeviceHS.pClassData != NULL);
}

bool MiniPC_Tx_Busy(void)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;
    return (hcdc == NULL) || (hcdc->TxState != 0U);
}

/* The buffer must stay untouched until MiniPC_Tx_Busy() returns false:
   the transfer runs asynchronously from it. */
uint8_t MiniPC_Transmit_Info(const uint8_t *buf, uint16_t len)
{
    if (!MiniPC_Is_Connected())
    {
        return USBD_FAIL;
    }
    return CDC_Transmit_HS((uint8_t *)buf, len);
}
