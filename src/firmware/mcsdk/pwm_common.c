/**
  ******************************************************************************
  * @file    pwm_common.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement common features
  *          of the PWM & Current Feedback component of the Motor Control SDK:
  *
  *           * start timers (main and auxiliary) synchronously
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
  * @ingroup pwm_curr_fdbk
  */

/* Includes ------------------------------------------------------------------*/
#include "pwm_common.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

#ifdef TIM2
/**
  * @brief  Performs the start of all the timers required by the control.
  * 
  * Uses TIM2 as a temporary timer to achieve synchronization between PWM signals.
  * When this function is called, TIM1 and/or TIM8 must be in a frozen state
  * with CNT, ARR, REP RATE and trigger correctly set (these settings are
  * usually performed in the Init method accordingly with the configuration)
  */
__weak void startTimers(void)
{
  uint32_t isTIM2ClockOn;
  uint32_t trigOut;

  isTIM2ClockOn = LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_TIM2);
  if ((uint32_t)0 == isTIM2ClockOn)
  {
    /* Temporary Enable TIM2 clock if not already on */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    LL_TIM_GenerateEvent_UPDATE(TIM2);
    LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM2);
  }
  else
  {
    trigOut = LL_TIM_ReadReg(TIM2, CR2) & TIM_CR2_MMS;
    LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_UPDATE);
    LL_TIM_GenerateEvent_UPDATE(TIM2);
    LL_TIM_SetTriggerOutput(TIM2, trigOut);
  }
}
#endif

/**
  * @brief  Waits for the end of the polarization. 
  * 
  * If the polarization exceeds the number of needed PWM cycles, it reports an error.
  * 
  * @param  TIMx Timer used to generate PWM.
  * @param  SWerror Variable used to report a SW error.
  * @param  repCnt Repetition counter value.
  * @param  cnt Polarization counter value.
  */
__weak void waitForPolarizationEnd(TIM_TypeDef *TIMx, uint16_t  *SWerror, uint8_t repCnt, volatile uint8_t *cnt)
{
#ifdef NULL_POW_COM
  if ((MC_NULL == cnt) || (MC_NULL == SWerror))
  {
    /* Nothing to do */
  }
  else
  {
#endif
    uint16_t hCalibrationPeriodCounter;
    uint16_t hMaxPeriodsNumber;

    hMaxPeriodsNumber = ((uint16_t)2 * NB_CONVERSIONS) * (((uint16_t)repCnt + 1U) >> 1);

    /* Wait for NB_CONVERSIONS to be executed */
    LL_TIM_ClearFlag_CC1(TIMx);
    hCalibrationPeriodCounter = 0u;
    while (*cnt < NB_CONVERSIONS)
    {
      if ((uint32_t)ERROR == LL_TIM_IsActiveFlag_CC1(TIMx))
      {
        LL_TIM_ClearFlag_CC1(TIMx);
        hCalibrationPeriodCounter++;
        if (hCalibrationPeriodCounter >= hMaxPeriodsNumber)
        {
          if (*cnt < NB_CONVERSIONS)
          {
            *SWerror = 1u;
            break;
          }
        }
      }
    }
#ifdef NULL_POW_COM
  }
#endif
  }

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
