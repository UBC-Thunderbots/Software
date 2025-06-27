/**
  ******************************************************************************
  * @file    current_ref_ctrl.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the
  *          six-step current mode current reference PWM generation component of 
  *          the Motor Control SDK.
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

/* Includes ------------------------------------------------------------------*/
#include "current_ref_ctrl.h"

/** @addtogroup MCSDK
  * @{
  */

/**
 * @defgroup current_ref_ctrl Six-Step, current mode, PWM generation for current reference
 *
 * @brief PWM generation of the current reference for Six-Step drive with current mode
 *
 * This implementation exploits a timer to generate a PWM as reference to limit the current
 * peak by means an external comparator 
 *
 * @{
 */
 
/**
  * @brief  It initializes TIMx
  * @param  pHandle: handler of instance of the CRM component
  * @retval none
  */
__weak void CRM_Init( CurrentRef_Handle_t * pHandle )
{
  switch (pHandle->pParams_str->RefTimerChannel)
  {
    case LL_TIM_CHANNEL_CH1:
    {
      LL_TIM_OC_SetCompareCH1(pHandle->pParams_str->TIMx, pHandle->StartCntPh);
    }
    break;    
    case LL_TIM_CHANNEL_CH2:
    {
      LL_TIM_OC_SetCompareCH2(pHandle->pParams_str->TIMx, pHandle->StartCntPh);
    }
    break;    
    case LL_TIM_CHANNEL_CH3:
    {
      LL_TIM_OC_SetCompareCH3(pHandle->pParams_str->TIMx, pHandle->StartCntPh);
    }
    break;    
    case LL_TIM_CHANNEL_CH4:
    {
      LL_TIM_OC_SetCompareCH4(pHandle->pParams_str->TIMx, pHandle->StartCntPh);
    }
    break;
  default:
      LL_TIM_OC_SetCompareCH1(pHandle->pParams_str->TIMx, pHandle->StartCntPh);
  }  
  LL_TIM_EnableCounter(pHandle->pParams_str->TIMx);
  LL_TIM_CC_EnableChannel(pHandle->pParams_str->TIMx, pHandle->pParams_str->RefTimerChannel);
}

/**
  * @brief  It clears TIMx counter and resets start-up duty cycle
  * @param  pHandle: handler of instance of the CRM component
  * @retval none
  */
__weak void CRM_Clear( CurrentRef_Handle_t * pHandle )
{
  LL_TIM_SetCounter(pHandle->pParams_str->TIMx, 0u);
  pHandle->Cnt = pHandle->StartCntPh;
  switch (pHandle->pParams_str->RefTimerChannel)
  {
    case LL_TIM_CHANNEL_CH1:
    {
      LL_TIM_OC_SetCompareCH1(pHandle->pParams_str->TIMx, pHandle->Cnt);
    }
    break;    
    case LL_TIM_CHANNEL_CH2:
    {
      LL_TIM_OC_SetCompareCH2(pHandle->pParams_str->TIMx, pHandle->Cnt);
    }
    break;    
    case LL_TIM_CHANNEL_CH3:
    {
      LL_TIM_OC_SetCompareCH3(pHandle->pParams_str->TIMx, pHandle->Cnt);
    }
    break;    
    case LL_TIM_CHANNEL_CH4:
    {
      LL_TIM_OC_SetCompareCH4(pHandle->pParams_str->TIMx, pHandle->Cnt);
    }
    break;
  default:
      LL_TIM_OC_SetCompareCH1(pHandle->pParams_str->TIMx, pHandle->Cnt);
  }  
}

/**
  * @brief  It updates current reference value
  * @param  pHandle: handler of instance of the CRM component
  * @param  hCnt: new reference value
  * @retval none
  */
__weak void CRM_SetReference( CurrentRef_Handle_t * pHandle, uint16_t hCnt )
{
  pHandle->Cnt = hCnt;
  switch (pHandle->pParams_str->RefTimerChannel)
  {
    case LL_TIM_CHANNEL_CH1:
    {
      LL_TIM_OC_SetCompareCH1(pHandle->pParams_str->TIMx, pHandle->Cnt);
    }
    break;    
    case LL_TIM_CHANNEL_CH2:
    {
      LL_TIM_OC_SetCompareCH2(pHandle->pParams_str->TIMx, pHandle->Cnt);
    }
    break;    
    case LL_TIM_CHANNEL_CH3:
    {
      LL_TIM_OC_SetCompareCH3(pHandle->pParams_str->TIMx, pHandle->Cnt);
    }
    break;    
    case LL_TIM_CHANNEL_CH4:
    {
      LL_TIM_OC_SetCompareCH4(pHandle->pParams_str->TIMx, pHandle->Cnt);
    }
    break;
  default:
      LL_TIM_OC_SetCompareCH1(pHandle->pParams_str->TIMx, pHandle->Cnt);
  }  
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
