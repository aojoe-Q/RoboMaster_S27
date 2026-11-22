/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : MiniPC.h
  * @brief          : MiniPC interfaces functions 
  * @author         : GrassFan Wang
  * @date           : 2025/02/10
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEVICE_MINIPC_H
#define DEVICE_MINIPC_H


/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h" 


#define MINIPC_TX_OK    0U      /* same value as USBD_OK */

/* Returns USBD_OK, USBD_BUSY or USBD_FAIL. */
uint8_t MiniPC_Transmit_Info(const uint8_t *buf, uint16_t len);

/* USB enumerated and the CDC class is up. */
bool MiniPC_Is_Connected(void);

/* A previous transmit is still in flight. */
bool MiniPC_Tx_Busy(void);
#endif