/**
  ******************************************************************************
  * @file    virtual_speed_sensor.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the Virtual Speed Sensor component of the Motor Control SDK.
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
  * @ingroup VirtualSpeedSensor
  */

/* Includes ------------------------------------------------------------------*/
#include "virtual_speed_sensor.h"


/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk
  * @{
  */

/** @defgroup VirtualSpeedSensor Virtual Speed & Position Feedback
  * @brief Virtual Speed Speed & Position Feedback implementation
  *
  * This component provides a "virtual" implementation of the speed and position feedback features.
  * This implementation provides a theoretical estimation of the speed and position of the rotor of
  * the motor based on a mechanical acceleration and an initial angle set by the application.
  *
  * This component is used during the @ref RevUpCtrl "Rev-Up Control" phases of the motor or in an
  * @ref OpenLoop "Open Loop Control" configuration in a sensorless subsystem.
  *
  *
  * @{
  */

/**
  * @brief  Software initialization of VirtualSpeedSensor component.
  * @param  pHandle: handler of the current instance of the VirtualSpeedSensor component.
  * @retval none
  *
  * - Calls VSS_Clear.
  * - Called at initialization of the whole MC core.
  */
__weak void VSS_Init(VirtualSpeedSensor_Handle_t *pHandle)
{
  VSS_Clear(pHandle);
}

/**
  * @brief  Software initialization of VSS object to be performed at each restart
  *         of the motor.
  * @param  pHandle: handler of the current instance of the VirtualSpeedSensor component.
  * @retval none
  */
__weak void VSS_Clear(VirtualSpeedSensor_Handle_t *pHandle)
{
#ifdef NULL_PTR_VIR_SPD_SEN
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->_Super.bSpeedErrorNumber = 0U;
    pHandle->_Super.hElAngle = 0;
    pHandle->_Super.hMecAngle = 0;
    pHandle->_Super.hAvrMecSpeedUnit = 0;
    pHandle->_Super.hElSpeedDpp = 0;
    pHandle->_Super.hMecAccelUnitP = 0;
    pHandle->_Super.bSpeedErrorNumber = 0U;

    pHandle->wElAccDppP32 = 0;
    pHandle->wElSpeedDpp32 = 0;
    pHandle->hRemainingStep = 0U;
    pHandle->hElAngleAccu = 0;

    pHandle->bTransitionStarted = false;
    pHandle->bTransitionEnded = false;
    pHandle->hTransitionRemainingSteps = pHandle->hTransitionSteps;
    pHandle->bTransitionLocked = false;

    pHandle->bCopyObserver = false;
#ifdef NULL_PTR_VIR_SPD_SEN
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
  * @brief  Updates the rotor electrical angle integrating the last settled
  *         instantaneous electrical speed express in [dpp](measurement_units.md).
  * @param  pHandle: handler of the current instance of the VirtualSpeedSensor component.
  * @retval int16_t Measured electrical angle in s16degree format.
  *
  * - Systematically called after #SPD_GetElAngle that retrieves last computed rotor electrical angle.
  */
__weak int16_t VSS_CalcElAngle(VirtualSpeedSensor_Handle_t *pHandle, int16_t *pInputVars_str)
{
  int16_t hRetAngle;
#ifdef NULL_PTR_VIR_SPD_SEN
  if ((MC_NULL == pHandle) || (MC_NULL == pInputVars_str))
  {
    hRetAngle = 0;
  }
  else
  {
#endif
    int16_t hAngleDiff;
    int32_t wAux;
    int16_t hAngleCorr;
    int16_t hSignCorr = 1;

    if (true == pHandle->bCopyObserver)
    {
      hRetAngle = *(int16_t *)pInputVars_str;
    }
    else
    {
      pHandle->hElAngleAccu += pHandle->_Super.hElSpeedDpp;
      pHandle->_Super.hMecAngle += (pHandle->_Super.hElSpeedDpp / (int16_t)pHandle->_Super.bElToMecRatio);

      if (true == pHandle->bTransitionStarted)
      {
        if (0 == pHandle->hTransitionRemainingSteps)
        {
          hRetAngle = *(int16_t *)pInputVars_str;
          pHandle->bTransitionEnded = true;
          pHandle->_Super.bSpeedErrorNumber = 0U;
        }
        else
        {
          pHandle->hTransitionRemainingSteps--;

          if (pHandle->_Super.hElSpeedDpp >= 0)
          {
            hAngleDiff = *(int16_t *)pInputVars_str - pHandle->hElAngleAccu;
          }
          else
          {
            hAngleDiff = pHandle->hElAngleAccu - *(int16_t *)pInputVars_str;
            hSignCorr = -1;
          }

          wAux = (int32_t)hAngleDiff * pHandle->hTransitionRemainingSteps;
          hAngleCorr = (int16_t)(wAux / pHandle->hTransitionSteps);
          hAngleCorr *= hSignCorr;

          if (hAngleDiff >= 0)
          {
            pHandle->bTransitionLocked = true;
            hRetAngle = *(int16_t *)pInputVars_str - hAngleCorr;
          }
          else
          {
            if (false == pHandle->bTransitionLocked)
            {
              hRetAngle = pHandle->hElAngleAccu;
            }
            else
            {
              hRetAngle = *(int16_t *)pInputVars_str + hAngleCorr;
            }
          }
        }
      }
      else
      {
        hRetAngle = pHandle->hElAngleAccu;
      }
    }

    pHandle->_Super.hElAngle = hRetAngle;
#ifdef NULL_PTR_VIR_SPD_SEN
  }
#endif
  return (hRetAngle);
}

/**
  * @brief  Computes and stores rotor instantaneous electrical speed (express
  *         in [dpp](measurement_units.md) considering the measurement frequency) in order to provide it
  *         to #SPD_GetElAngle.
  * @param  pHandle: handler of the current instance of the VirtualSpeedSensor component.
  * @param  hMecSpeedUnit pointer to int16_t, used to return the rotor average
  *         mechanical speed [SPEED_UNIT](measurement_units.md).
  * @retval bool true = sensor information is reliable and false = sensor information is not reliable.
  *
  * - Stores and returns through parameter hMecSpeedUnit the rotor average mechanical speed,
  *  expressed in the unit defined by [SPEED_UNIT](measurement_units.md).
  * - Returns the reliability state of the sensor (always true).
  * - Called with the same periodicity on which speed control is executed, precisely during START and SWITCH_OVER states
  * of the MC tasks state machine or in its RUM state in @ref OpenLoop "Open Loop Control" configuration into TSK_MediumFrequencyTask.
  */
__weak bool VSS_CalcAvrgMecSpeedUnit(VirtualSpeedSensor_Handle_t *pHandle, int16_t *hMecSpeedUnit)
{
  bool SpeedSensorReliability;
#ifdef NULL_PTR_VIR_SPD_SEN
  if ((MC_NULL == pHandle) || (MC_NULL == hMecSpeedUnit))
  {
    SpeedSensorReliability = false;
  }
  else
  {
#endif
    if (pHandle->hRemainingStep > 1u)
    {
      pHandle->wElSpeedDpp32 += pHandle->wElAccDppP32;
#ifndef FULL_MISRA_C_COMPLIANCY_VIRT_SPD_SENS
      //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
      pHandle->_Super.hElSpeedDpp = (int16_t)(pHandle->wElSpeedDpp32 >> 16);
#else
      pHandle->_Super.hElSpeedDpp = (int16_t)(pHandle->wElSpeedDpp32 / 65536);
#endif

      /* Convert dpp into MecUnit */
      *hMecSpeedUnit = (int16_t)((((int32_t)pHandle->_Super.hElSpeedDpp)
                               * ((int32_t )pHandle->_Super.hMeasurementFrequency) * SPEED_UNIT)
                               / (((int32_t)pHandle->_Super.DPPConvFactor) * ((int32_t)pHandle->_Super.bElToMecRatio)));
      pHandle->_Super.hAvrMecSpeedUnit = *hMecSpeedUnit;
      pHandle->hRemainingStep--;
    }
    else if (1U == pHandle->hRemainingStep)
    {
      *hMecSpeedUnit = pHandle->hFinalMecSpeedUnit;
      pHandle->_Super.hAvrMecSpeedUnit = *hMecSpeedUnit;
      pHandle->_Super.hElSpeedDpp = (int16_t)((((int32_t)*hMecSpeedUnit) * ((int32_t)pHandle->_Super.DPPConvFactor))
                                            / (((int32_t)SPEED_UNIT) * ((int32_t)pHandle->_Super.hMeasurementFrequency)));
      pHandle->_Super.hElSpeedDpp *= ((int16_t)pHandle->_Super.bElToMecRatio);
      pHandle->hRemainingStep = 0U;
    }
    else
    {
      *hMecSpeedUnit = pHandle->_Super.hAvrMecSpeedUnit;
    }
    /* If the transition is not done yet, we already know that speed is not reliable */
    if (false == pHandle->bTransitionEnded)
    {
      pHandle->_Super.bSpeedErrorNumber = pHandle->_Super.bMaximumSpeedErrorsNumber;
      SpeedSensorReliability = false;
    }
    else
    {
      SpeedSensorReliability = SPD_IsMecSpeedReliable(&pHandle->_Super, hMecSpeedUnit);
    }
#ifdef NULL_PTR_VIR_SPD_SEN
  }
#endif
  return (SpeedSensorReliability);
}

/**
  * @brief  Sets instantaneous information on VSS mechanical and electrical angle.
  * @param  pHandle: handler of the current instance of the VirtualSpeedSensor component.
  * @param  hMecAngle: instantaneous measure of rotor mechanical angle.
  * @retval none
  *
  * - Called during @ref RevUpCtrl "Rev-Up Control" and
  * @ref EncAlignCtrl "Encoder Alignment Controller procedure" initialization.
  */
__weak void VSS_SetMecAngle(VirtualSpeedSensor_Handle_t *pHandle, int16_t hMecAngle)
{
#ifdef NULL_PTR_VIR_SPD_SEN
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->hElAngleAccu = hMecAngle;
    pHandle->_Super.hMecAngle = pHandle->hElAngleAccu / ((int16_t)pHandle->_Super.bElToMecRatio);
    pHandle->_Super.hElAngle = hMecAngle;
#ifdef NULL_PTR_VIR_SPD_SEN
  }
#endif
}

/**
  * @brief  Sets the mechanical acceleration of virtual speed sensor.
  * @param  pHandle: handler of the current instance of the VirtualSpeedSensor component.
  * @param  hFinalMecSpeedUnit mechanical speed assumed by
  *         the virtual speed sensor at the end of the duration. Expressed in the unit defined
  *         by [SPEED_UNIT](measurement_units.md).
  * @param  hDurationms: Duration expressed in ms. It can be 0 to apply
  *         instantaneous the final speed.
  * @retval none
  *
  * - This acceleration is defined starting from current mechanical speed, final mechanical
  * speed expressed in [SPEED_UNIT](measurement_units.md) and duration expressed in milliseconds.
  * - Called during @ref RevUpCtrl "Rev-Up Control" and
  * @ref EncAlignCtrl "Encoder Alignment Controller procedure" initialization.
  */
__weak void  VSS_SetMecAcceleration(VirtualSpeedSensor_Handle_t *pHandle, int16_t hFinalMecSpeedUnit,
                                    uint16_t hDurationms)
{
#ifdef NULL_PTR_VIR_SPD_SEN
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    int32_t wMecAccDppP32;
    uint16_t hNbrStep;
    int16_t hCurrentMecSpeedDpp;
    int16_t hFinalMecSpeedDpp;

    if (false == pHandle->bTransitionStarted)
    {
      if (0U == hDurationms)
      {
        pHandle->_Super.hAvrMecSpeedUnit = hFinalMecSpeedUnit;

        pHandle->_Super.hElSpeedDpp = (int16_t)((((int32_t)hFinalMecSpeedUnit)
                                               * ((int32_t)pHandle->_Super.DPPConvFactor))
                                              / (((int32_t)SPEED_UNIT)
                                               * ((int32_t)pHandle->_Super.hMeasurementFrequency)));

        pHandle->_Super.hElSpeedDpp *= ((int16_t)pHandle->_Super.bElToMecRatio);

        pHandle->hRemainingStep = 0U;

        pHandle->hFinalMecSpeedUnit = hFinalMecSpeedUnit;
      }
      else
      {
        hNbrStep = (uint16_t)((((uint32_t)hDurationms) * ((uint32_t)pHandle->hSpeedSamplingFreqHz)) / 1000U);
        hNbrStep++;
        pHandle->hRemainingStep = hNbrStep;
        hCurrentMecSpeedDpp = pHandle->_Super.hElSpeedDpp / ((int16_t)pHandle->_Super.bElToMecRatio);
        hFinalMecSpeedDpp = (int16_t)((((int32_t )hFinalMecSpeedUnit) * ((int32_t)pHandle->_Super.DPPConvFactor))
                                    / (((int32_t )SPEED_UNIT) * ((int32_t)pHandle->_Super.hMeasurementFrequency)));

        if (0U == hNbrStep)
        {
          /* Nothing to do */
        }
        else
        {
          wMecAccDppP32 = ((((int32_t)hFinalMecSpeedDpp) - ((int32_t)hCurrentMecSpeedDpp))
                         * ((int32_t)65536)) / ((int32_t )hNbrStep);

          pHandle->wElAccDppP32 = wMecAccDppP32 * ((int16_t)pHandle->_Super.bElToMecRatio);
        }

        pHandle->hFinalMecSpeedUnit = hFinalMecSpeedUnit;

        pHandle->wElSpeedDpp32 = ((int32_t)pHandle->_Super.hElSpeedDpp) * ((int32_t)65536);
      }
    }
#ifdef NULL_PTR_VIR_SPD_SEN
  }
#endif
}

/**
  * @brief  Checks if the ramp executed after a #VSS_SetMecAcceleration command
  *         has been completed by checking zero value of the Number of steps remaining to reach the final
  *         speed.
  * @param  pHandle: handler of the current instance of the VirtualSpeedSensor component.
  * @retval bool: true if the ramp is completed, otherwise false.
  *
  * - Not used into current implementation.
  */
__weak bool VSS_RampCompleted(VirtualSpeedSensor_Handle_t *pHandle)
{
  bool retVal = false;
#ifdef NULL_PTR_VIR_SPD_SEN
  if (MC_NULL == pHandle)
  {
    /* nothing to do */
  }
  else
  {
#endif
    if (0U == pHandle->hRemainingStep)
    {
      retVal = true;
    }
    else
    {
      /* Nothing to do */
    }
#ifdef NULL_PTR_VIR_SPD_SEN
  }
#endif
  return (retVal);
}

/**
  * @brief  Gets the final speed of last settled ramp of virtual speed sensor expressed
            in [SPEED_UNIT](measurement_units.md).
  * @param  pHandle: handler of the current instance of the VirtualSpeedSensor component.
  * @retval none
  *
  * - Will be call for future dual motor implementation into START state of MC tasks state machine into TSK_MediumFrequencyTask.
  */
__weak int16_t  VSS_GetLastRampFinalSpeed(VirtualSpeedSensor_Handle_t *pHandle)
{
#ifdef NULL_PTR_VIR_SPD_SEN
  return ((MC_NULL == pHandle) ? 0 : pHandle->hFinalMecSpeedUnit);
#else
  return (pHandle->hFinalMecSpeedUnit);
#endif
}

/**
  * @brief  Sets the command to Start the transition phase from Virtual Speed Sensor
  *         to other Speed Sensor.
  * @param  pHandle: handler of the current instance of the VirtualSpeedSensor component.
  * @param  bool: true to Start the transition phase, false has no effect.
  * @retval bool: true if Transition phase is enabled (started or not), false if
  *         transition has been triggered but it's actually disabled
  *         (parameter #hTransitionSteps = 0).
  *
  * - Transition is to be considered ended when Sensor information is
  *  declared 'Reliable' or if function returned value is false.
  * - Called into START state of MC tasks state machine into TSK_MediumFrequencyTask.
  */
__weak bool VSS_SetStartTransition(VirtualSpeedSensor_Handle_t *pHandle, bool bCommand)
{
  bool bAux = true;
#ifdef NULL_PTR_VIR_SPD_SEN
  if (MC_NULL == pHandle)
  {
    /* nothing to do */
  }
  else
  {
#endif
    if (true == bCommand)
    {
      pHandle->bTransitionStarted = true;

      if (0 == pHandle->hTransitionSteps)
      {
        pHandle->bTransitionEnded = true;
        pHandle->_Super.bSpeedErrorNumber = 0U;
        bAux = false;
      }
    }
#ifdef NULL_PTR_VIR_SPD_SEN
  }
#endif
  return (bAux);
}

/**
  * @brief  Returns the status of the transition phase by checking the status of the two parameters
  *         @ref VirtualSpeedSensor_Handle_t::bTransitionStarted "bTransitionStarted" and
  *         @ref VirtualSpeedSensor_Handle_t::bTransitionEnded "bTransitionEnded".
  * @param  pHandle: handler of the current instance of the VirtualSpeedSensor components
  * @retval bool: true if Transition phase is ongoing, false otherwise.
  *
  * - Not used into current implementation.
  */
__weak bool VSS_IsTransitionOngoing(VirtualSpeedSensor_Handle_t *pHandle)
{
  bool retVal = false;
#ifdef NULL_PTR_VIR_SPD_SEN
  if (MC_NULL == pHandle)
  {
    /* nothing to do */
  }
  else
  {
#endif
    uint16_t hTS = 0U;
    uint16_t hTE = 0U;
    uint16_t hAux;

    if (true == pHandle->bTransitionStarted)
    {
      hTS = 1U;
    }
    if (true == pHandle->bTransitionEnded)
    {
      hTE = 1U;
    }
    hAux = hTS ^ hTE;
    if (hAux != 0U)
    {
      retVal = true;
    }
#ifdef NULL_PTR_VIR_SPD_SEN
  }
#endif
  return (retVal);
}

/**
  * @brief  Returns the ending status of the transition phase by checking the parameter
  *         @ref VirtualSpeedSensor_Handle_t::bTransitionEnded "bTransitionEnded".
  * @param  pHandle: handler of the current instance of the VirtualSpeedSensor components
  * @retval bool: true if Transition phase ended, false otherwise.
  *
  * - Called into SWITCH_OVER state of MC tasks state machine into TSK_MediumFrequencyTask.
  */
__weak bool VSS_TransitionEnded(VirtualSpeedSensor_Handle_t *pHandle)
{
#ifdef NULL_PTR_VIR_SPD_SEN
  return ((MC_NULL == pHandle) ? false : pHandle->bTransitionEnded);
#else
  return (pHandle->bTransitionEnded);
#endif
}

/**
  * @brief  Sets instantaneous information on rotor electrical angle same as copied by state observer.
  * @param  pHandle: handler of the current instance of the VirtualSpeedSensor component.
  * @retval none
  *
  * - Not used into current implementation.
  */
__weak void VSS_SetCopyObserver(VirtualSpeedSensor_Handle_t *pHandle)
{
#ifdef NULL_PTR_VIR_SPD_SEN
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->bCopyObserver = true;
#ifdef NULL_PTR_VIR_SPD_SEN
  }
#endif
}

/**
  * @brief  Sets instantaneous information on rotor electrical angle.
  * @param  pHandle: handler of the current instance of the VirtualSpeedSensor component.
  * @param  hElAngle instantaneous measure of rotor electrical angle in [s16degrees](measurement_units.md).
  * @retval none
  *
  * - Not used into current implementation.
  */
__weak void VSS_SetElAngle(VirtualSpeedSensor_Handle_t *pHandle, int16_t hElAngle)
{
#ifdef NULL_PTR_VIR_SPD_SEN
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->hElAngleAccu = hElAngle;
    pHandle->_Super.hElAngle = hElAngle;
#ifdef NULL_PTR_VIR_SPD_SEN
  }
#endif
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
