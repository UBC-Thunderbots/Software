/**
  ******************************************************************************
  * @file    digital_output.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          digital output component of the Motor Control SDK.
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
  * @ingroup DigitalOutput
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DIGITALOUTPUT_H
#define DIGITALOUTPUT_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup DigitalOutput
  * @{
  */


/* Exported constants --------------------------------------------------------*/
#define DOutputActiveHigh       1U /*!< Digital output active high flag */
#define DOutputActiveLow        0U /*!< Digital output active low flag */


/* Exported types ------------------------------------------------------------*/

/**
  * @brief Digital output handler definition
  */
typedef struct
{
  DOutputState_t OutputState;       /*!< indicates the state of the digital output */
  GPIO_TypeDef *hDOutputPort;       /*!< GPIO output port. It must be equal
                                         to GPIOx x= A, B, ...*/
  uint16_t hDOutputPin;             /*!< GPIO output pin. It must be equal to
                                         GPIO_Pin_x x= 0, 1, ...*/
  uint8_t  bDOutputPolarity;        /*!< GPIO output polarity. It must be equal
                                          to DOutputActiveHigh or DOutputActiveLow */
} DOUT_handle_t;


/* Accordingly with selected polarity, it sets to active or inactive the
 * digital output
 */
void DOUT_SetOutputState(DOUT_handle_t *pHandle, DOutputState_t State);

/* It returns the state of the digital output */
__weak DOutputState_t DOUT_GetOutputState(DOUT_handle_t *pHandle);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* DIGITALOUTPUT_H */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
