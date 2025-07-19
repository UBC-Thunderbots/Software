/**
  ******************************************************************************
  * @file    current_ref_ctrl.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          six-step current mode current reference PWM generation component of 
  *          the Motor Control SDK.
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
  * @ingroup current_ref_ctrl
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CURRENTREF_H
#define CURRENTREF_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup current_ref_ctrl
  * @{
  */
/* Exported defines ------------------------------------------------------------*/

/* Exported defines ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  CurrentRef pulse polarity definition
  */
typedef enum
{
  UP = 0,
  DOWN = 1,
} CurrentRef_PulsePolarity;

/**
  * @brief  CurrentRef parameters definition
  */
typedef struct
{
  TIM_TypeDef * TIMx;                  /*!< It contains the pointer to the timer
                                           used for current reference PWM generation. */
  uint32_t RefTimerChannel;            /*!< Channel of the timer used the generation */ 
} CurrentRef_Params_t;

/**
  * @brief This structure is used to handle the data of an instance of the Current Reference component
  *
  */
typedef struct
{
  CurrentRef_Params_t const * pParams_str;

  uint16_t Cnt;                                    /**< PWM Duty cycle */
  uint16_t StartCntPh;
  uint16_t PWMperiod;                                  /**< PWM period expressed in timer clock cycles unit:
                                                          *  @f$hPWMPeriod = TimerFreq_{CLK} / F_{PWM}@f$    */
  CurrentRef_PulsePolarity pPolarity;
} CurrentRef_Handle_t;

/* Exported functions --------------------------------------------------------*/

void CRM_Init( CurrentRef_Handle_t * pHandle );

void CRM_Clear ( CurrentRef_Handle_t * pHandle );

void CRM_SetReference( CurrentRef_Handle_t * pHandle, uint16_t hCnt );

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* CURRENTREF_H */

/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
