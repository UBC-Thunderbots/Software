/**
  ******************************************************************************
  * @file    open_loop.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the Open Loop component.
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
  * @ingroup OpenLoop
  */
  
/* Includes ------------------------------------------------------------------*/
#include "open_loop.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup OpenLoop Open Loop Control
  * @brief Open Loop component of the Motor Control SDK
  *
  * Open Loop component allows to run the motor in open loop voltage mode. In that mode, the phase voltages are
  * forced independently from the measured currents. To do so, the routine OL_VqdConditioning() overwrites the 
  * voltage command Vdq in the FOC current controller task. The voltage level to apply can be set directly by the
  * user, with OL_UpdateVoltage(), or computed by OL_Calc() if the V/F mode is selected. In that mode, the voltage 
  * level depends on the speed, the slope and the offset selected by the user. 
  *
  * @{
  */

/* Private defines -----------------------------------------------------------*/

/**
  * @brief  Initializes OpenLoop variables.it should be called 
  *         once during Motor Control initialization.
  * @param  pHandle: Pointer on Handle structure of OpenLoop feature.
  * @param  pVSS: Pointer on virtual speed sensor structure.
  */
__weak void OL_Init(OpenLoop_Handle_t *pHandle, VirtualSpeedSensor_Handle_t *pVSS)
{
#ifdef NULL_PTR_CHECK_OPEN_LOOP
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->hVoltage = pHandle->hDefaultVoltage;
    pHandle->pVSS = pVSS;
#ifdef NULL_PTR_CHECK_OPEN_LOOP
  }
#endif
}

/**
  * @brief  Sets Vqd according to open loop phase voltage. It should be 
  *         called during current controller task.
  * @param  pHandle: Pointer on Handle structure of OpenLoop feature.
  * @retval qd_t Vqd conditioned values.
  */
__weak qd_t OL_VqdConditioning(const OpenLoop_Handle_t *pHandle)
{
  qd_t Vqd;
  Vqd.d = 0;
#ifdef NULL_PTR_CHECK_OPEN_LOOP
  Vqd.q = ((MC_NULL == pHandle) ? 0 : pHandle->hVoltage);
#else
  Vqd.q = (pHandle->hVoltage);
#endif
  return (Vqd);
}

/**
  * @brief  Sets new open loop phase voltage.
  * @param  pHandle: Pointer on Handle structure of OpenLoop feature.
  * @param  hNewVoltage: New voltage value to apply.
  */
__weak void OL_UpdateVoltage(OpenLoop_Handle_t *pHandle, int16_t hNewVoltage)
{
#ifdef NULL_PTR_CHECK_OPEN_LOOP
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->hVoltage = hNewVoltage;
#ifdef NULL_PTR_CHECK_OPEN_LOOP
  }
#endif
}

/**
  * @brief  Gets open loop phase voltage.
  * @param  pHandle: Pointer on Handle structure of OpenLoop feature.
  */
__weak int16_t OL_GetVoltage(OpenLoop_Handle_t *pHandle)
{
  int16_t hVoltage;
#ifdef NULL_PTR_CHECK_OPEN_LOOP
  hVoltage = ((MC_NULL == pHandle) ? 0 : pHandle->hVoltage);
#else
  hVoltage = pHandle->hVoltage;
#endif
  return (hVoltage);
}

/**
  * @brief  Computes phase voltage to apply according to average mechanical speed (V/F Mode).
  *         It should be called during background task.
  * @param  pHandle: Pointer on Handle structure of OpenLoop feature.
  */
__weak void OL_Calc(OpenLoop_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_OPEN_LOOP
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    if (true ==  pHandle->VFMode)
    {
      /* V/F mode true means enabled */
      if (pHandle->pVSS->_Super.hAvrMecSpeedUnit >= 0)
      {
        pHandle->hVoltage = pHandle->hVFOffset + (pHandle->hVFSlope * pHandle->pVSS->_Super.hAvrMecSpeedUnit);
      }
      else
      {
        pHandle->hVoltage = pHandle->hVFOffset - (pHandle->hVFSlope * pHandle->pVSS->_Super.hAvrMecSpeedUnit);
      }
    }
#ifdef NULL_PTR_CHECK_OPEN_LOOP
  }
#endif
}

/**
  * @brief  Activates of the Voltage versus Frequency mode (V/F mode).
  * @param  pHandle: Pointer on Handle structure of OpenLoop feature.
  * @param  VFEnabling: Flag to enable the V/F mode.
  */
__weak void OL_VF(OpenLoop_Handle_t *pHandle, bool VFEnabling)
{
#ifdef NULL_PTR_CHECK_OPEN_LOOP
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->VFMode = VFEnabling;
#ifdef NULL_PTR_CHECK_OPEN_LOOP
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
