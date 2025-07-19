/**
  ******************************************************************************
  * @file    inrush_current_limiter.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions implementing the
  *          Inrush Current Limiter feature of the Motor Control SDK.
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
  * @ingroup ICL
  */
  
/* Includes ------------------------------------------------------------------*/
#include "inrush_current_limiter.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup ICL Inrush Current Limiter
  * @brief Inrush Current Limiter component of the Motor Control SDK
  *
  * Inrush current limiter acts on the NTC bypass relay according to the DC bus level in 
  * order to limit the current when charging DC bulk capacitor.
  * In order to work properly, it needs to point on the Vbus and digital output handlers
  * to measure the DC bus and control the NTC bypass relay. Inrush current limiter has 
  * its own state machine described below:
  *
  * ![](ICL_FSM.svg)
  *
  * @{
  */


/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes all the needed ICL component variables.
  *         It shall be called only once during Motor Control initialization.
  * @param  pHandle: handler of the current instance of the ICL component
  * @param  pVBS the bus voltage sensor used by the ICL.
  * @param  pDOUT the digital output used by the ICL.
  */
__weak void ICL_Init(ICL_Handle_t *pHandle, BusVoltageSensor_Handle_t *pVBS, DOUT_handle_t *pDOUT)
{
  pHandle->pVBS = pVBS;
  pHandle->pDOUT = pDOUT;
  DOUT_SetOutputState(pDOUT, ACTIVE);
}

/**
  * @brief  Executes the inrush current limiter state machine and shall be 
  *         called during background task.
  * @param  pHandle handler of the current instance of the ICL component
  * @retval ICLState_t returns the current ICL state machine
  */
__weak ICL_State_t ICL_Exec(ICL_Handle_t *pHandle)
{
  /* ICL actions.*/
  switch (pHandle->ICLstate)
  {
    case ICL_ACTIVATION:
    {
      /* ICL activation: counting the step before pass in ICL_ACTIVE */
      if (pHandle->hICLTicksCounter == 0u)
      {
        pHandle->ICLstate = ICL_ACTIVE;
        pHandle->hICLTicksCounter = pHandle->hICLChargingDelayTicks;        
      }
      else
      {
        pHandle->hICLTicksCounter--;
      }
    }
    break;

    case ICL_DEACTIVATION:
    {
      /* ICL deactivation: counting the step before pass in ICL_INACTIVE.*/
      if (pHandle->hICLTicksCounter == 0u)
      {
        pHandle->ICLstate = ICL_INACTIVE;
      }
      else
      {
        pHandle->hICLTicksCounter--;
      }
    }
    break;

    case ICL_ACTIVE:
    {
  
      /* ICL is active: if bus is present and capacitor charging time elapsed*/
      if (pHandle->hICLTicksCounter == 0u)
      {
        if (VBS_GetAvBusVoltage_d(pHandle->pVBS) > pHandle->hICLVoltageThreshold){
          DOUT_SetOutputState(pHandle->pDOUT, INACTIVE);
          pHandle->ICLstate = ICL_DEACTIVATION;
          pHandle->hICLTicksCounter = pHandle->hICLSwitchDelayTicks;
        }
      }
      else
      {
          pHandle->hICLTicksCounter--;
      }
    }
    break;

    case ICL_INACTIVE:
    {
      /* ICL is inactive: if bus is not present activate the ICL */
      if (VBS_CheckVbus(pHandle->pVBS) == MC_UNDER_VOLT)
      {
        DOUT_SetOutputState(pHandle->pDOUT, ACTIVE);
        pHandle->ICLstate = ICL_ACTIVATION;
        pHandle->hICLTicksCounter = pHandle->hICLSwitchDelayTicks;
      }
    }
    break;

    case ICL_IDLE:
    default:
    {
    }
    break;
  }

  return pHandle->ICLstate;
}


/**
  * @brief  Returns the current state of the ICL state machine.
  * @param  pHandle: handler of the current instance of the ICL component
  * @retval ICLState_t returns the current ICL state machine
  */
__weak ICL_State_t ICL_GetState(ICL_Handle_t *pHandle)
{
  return pHandle->ICLstate;
}


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
