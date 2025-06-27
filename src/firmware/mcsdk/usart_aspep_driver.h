/**
  ******************************************************************************
  * @file    usart_aspep_driver.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *           uart driver for the aspep protocol.
  *
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
#ifndef usart_aspep_driver_h
#define usart_aspep_driver_h

#include <stdint.h>
#include <stdbool.h>

/* To be removed no protocol awarness at this level */

typedef struct
{
  USART_TypeDef *USARTx;
  DMA_TypeDef *rxDMA;
  DMA_TypeDef *txDMA;
  uint32_t rxChannel;
  uint32_t txChannel;
} UASPEP_Handle_t;

bool UASPEP_SEND_PACKET(void *pHWHandle, void *data, uint16_t length);
void UASPEP_RECEIVE_BUFFER(void *pHWHandle, void *buffer, uint16_t length);
void UASPEP_INIT(void *pHWHandle);
void UASPEP_IDLE_ENABLE(void *pHWHandle);

#endif
/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
