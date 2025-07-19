/**
  ******************************************************************************
  * @file    ramp_ext_mngr.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the Ramp Extended Manager component of the Motor Control SDK:
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
#include "ramp_ext_mngr.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup RampExtMngr Ramp Manager
  * @brief Ramp Extended Manager component of the Motor Control SDK
  *
  * @todo Document the Ramp Extended Manager "module".
  *
  * @{
  */
/* Private function prototypes -----------------------------------------------*/
uint32_t getScalingFactor(int32_t Target);

/**
  * @brief  It reset the state variable to zero.
  * @param  pHandle related Handle of struct RampMngr_Handle_t
  * @retval none.
  */
void REMNG_Init(RampExtMngr_Handle_t *pHandle)
{
#ifdef NULL_RMP_EXT_MNG
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->Ext = 0;
    pHandle->TargetFinal = 0;
    pHandle->RampRemainingStep = 0U;
    pHandle->IncDecAmount = 0;
    pHandle->ScalingFactor = 1U;
#ifdef NULL_RMP_EXT_MNG
  }
#endif
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
  * @brief  Exec the ramp calculations and returns the current value of the
            state variable.
            It must be called at fixed interval defined in the hExecFreq.
  * @param  pHandle related Handle of struct RampMngr_Handle_t
  * @retval int32_t value of the state variable
  */
__weak int32_t REMNG_Calc(RampExtMngr_Handle_t *pHandle)
{
  int32_t ret_val;
#ifdef NULL_RMP_EXT_MNG
  if (MC_NULL == pHandle)
  {
    ret_val = 0;
  }
  else
  {
#endif
    int32_t current_ref;

    current_ref = pHandle->Ext;

    /* Update the variable and terminates the ramp if needed. */
    if (pHandle->RampRemainingStep > 1U)
    {
      /* Increment/decrement the reference value. */
      current_ref += pHandle->IncDecAmount;

      /* Decrement the number of remaining steps */
      pHandle->RampRemainingStep --;
    }
    else if (1U == pHandle->RampRemainingStep)
    {
      /* Set the backup value of TargetFinal. */
      current_ref = pHandle->TargetFinal * ((int32_t)pHandle->ScalingFactor);
      pHandle->RampRemainingStep = 0U;
    }
    else
    {
      /* Do nothing. */
    }

    pHandle->Ext = current_ref;
    ret_val = pHandle->Ext / ((int32_t)pHandle->ScalingFactor);
#ifdef NULL_RMP_EXT_MNG
  }
#endif
  return (ret_val);
}

/**
  * @brief  Setup the ramp to be executed
  * @param  pHandle related Handle of struct RampMngr_Handle_t
  * @param  hTargetFinal (signed 32bit) final value of state variable at the end
  *         of the ramp.
  * @param  hDurationms (unsigned 32bit) the duration of the ramp expressed in
  *         milliseconds. It is possible to set 0 to perform an instantaneous
  *         change in the value.
  * @retval bool It returns true is command is valid, false otherwise
  */
__weak bool REMNG_ExecRamp(RampExtMngr_Handle_t *pHandle, int32_t TargetFinal, uint32_t Durationms)
{
  bool retVal = true;
#ifdef NULL_RMP_EXT_MNG
  if (MC_NULL == pHandle)
  {
    retVal = false;
  }
  else
  {
#endif
    uint32_t aux;
    int32_t aux1;
    int32_t current_ref;


    /* Get current state */
    current_ref = pHandle->Ext / ((int32_t)pHandle->ScalingFactor);

    if (0U == Durationms)
    {
      pHandle->ScalingFactor = getScalingFactor(TargetFinal);
      pHandle->Ext = TargetFinal * ((int32_t)pHandle->ScalingFactor);
      pHandle->RampRemainingStep = 0U;
      pHandle->IncDecAmount = 0;
    }
    else
    {
      uint32_t wScalingFactor = getScalingFactor(TargetFinal - current_ref);
      uint32_t wScalingFactor2 = getScalingFactor(current_ref);
      uint32_t wScalingFactor3 = getScalingFactor(TargetFinal);
      uint32_t wScalingFactorMin;

      if (wScalingFactor <  wScalingFactor2)
      {
        if (wScalingFactor < wScalingFactor3)
        {
          wScalingFactorMin = wScalingFactor;
        }
        else
        {
          wScalingFactorMin = wScalingFactor3;
        }
      }
      else
      {
        if (wScalingFactor2 < wScalingFactor3)
        {
          wScalingFactorMin = wScalingFactor2;
        }
        else
        {
          wScalingFactorMin = wScalingFactor3;
        }
      }

      pHandle->ScalingFactor = wScalingFactorMin;
      pHandle->Ext = current_ref * ((int32_t)pHandle->ScalingFactor);

      /* Store the TargetFinal to be applied in the last step */
      pHandle->TargetFinal = TargetFinal;

      /* Compute the (wRampRemainingStep) number of steps remaining to complete
      the ramp. */
      aux = Durationms * ((uint32_t)pHandle->FrequencyHz); /* Check for overflow and use prescaler */
      aux /= 1000U;
      pHandle->RampRemainingStep = aux;
      pHandle->RampRemainingStep++;

      /* Compute the increment/decrement amount (wIncDecAmount) to be applied to
      the reference value at each CalcTorqueReference. */
      aux1 = (TargetFinal - current_ref) * ((int32_t)pHandle->ScalingFactor);
      aux1 /= ((int32_t)pHandle->RampRemainingStep);
      pHandle->IncDecAmount = aux1;
    }
#ifdef NULL_RMP_EXT_MNG
  }
#endif
  return retVal;
}

/**
  * @brief  Returns the current value of the state variable.
  * @param  pHandle related Handle of struct RampMngr_Handle_t
  * @retval int32_t value of the state variable
  */
__weak int32_t REMNG_GetValue(const RampExtMngr_Handle_t *pHandle)
{
#ifdef NULL_RMP_EXT_MNG
  return ((MC_NULL == pHandle) ? 0 : (pHandle->Ext / ((int32_t)pHandle->ScalingFactor)));
#else
  return (pHandle->Ext / ((int32_t)pHandle->ScalingFactor));
#endif
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
  * @brief  Check if the settled ramp has been completed.
  * @param  pHandle related Handle of struct RampMngr_Handle_t.
  * @retval bool It returns true if the ramp is completed, false otherwise.
  */
__weak bool REMNG_RampCompleted(const RampExtMngr_Handle_t *pHandle)
{
  bool retVal = false;
#ifdef NULL_RMP_EXT_MNG
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    if (0U == pHandle->RampRemainingStep)
    {
      retVal = true;
    }
    else
    {
      /* nothing to do */
    }
#ifdef NULL_RMP_EXT_MNG
  }
#endif
  return (retVal);

}

/**
  * @brief  Stop the execution of the ramp keeping the last reached value.
  * @param  pHandle related Handle of struct RampMngr_Handle_t.
  * @retval none
  */
__weak void REMNG_StopRamp(RampExtMngr_Handle_t *pHandle)
{
#ifdef NULL_RMP_EXT_MNG
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->RampRemainingStep = 0U;
    pHandle->IncDecAmount = 0;
#ifdef NULL_RMP_EXT_MNG
  }
#endif
}

/**
  * @brief  Calculating the scaling factor to maximixe the resolution. It
  *         perform the 2^int(31-log2(Target)) with an iterative approach.
  *         It allows to keep Target * Scaling factor inside int32_t type.
  * @param  Target Input data.
  * @retval uint32_t It returns the optimized scaling factor.
  */
__weak uint32_t getScalingFactor(int32_t Target)
{
  uint32_t TargetAbs;
  int32_t aux;
  uint8_t i;

  if (Target < 0)
  {
    aux = -Target;
    TargetAbs = (uint32_t)aux;
  }
  else
  {
    TargetAbs = (uint32_t)Target;
  }
  for (i = 1U; i < 32U; i++)
  {
    uint32_t limit = (((uint32_t)1) << (31U - i));
    if (TargetAbs >= limit)
    {
      break;
    }
    else
    {
      /* Nothing to do */
    }
  }
  return (((uint32_t)1) << (i - 1U));
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
