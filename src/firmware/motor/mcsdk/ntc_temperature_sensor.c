/**
  ******************************************************************************
  * @file    ntc_temperature_sensor.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the Temperature Sensor component of the Motor Control SDK.
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
  * @ingroup TemperatureSensor
  */

/* Includes ------------------------------------------------------------------*/
#include "ntc_temperature_sensor.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup TemperatureSensor NTC Temperature Sensor
  * @brief Allows to read the temperature of the heat sink
  *
  * This component implements both a virtual and a real temperature sensor,
  * depending on the sensor availability.
  *
  * Access to the MCU peripherals needed to acquire the temperature (GPIO and ADC
  * used for regular conversion) is managed by the PWM component used in the Motor
  * Control subsystem. As a consequence, this NTC temperature sensor implementation
  * is hardware-independent.
  *
  * If a real temperature sensor is available (Sensor Type = #REAL_SENSOR),
  * this component can handle NTC sensors or, more generally, analog temperature sensors
  * which output is related to the temperature by the following formula:
  *
  * @f[
  *               V_{out} = V_0 + \frac{dV}{dT} \cdot ( T - T_0)
  * @f]
  *
  * In case of Pull up configuration @f$\frac{dV}{dT}@f$ is positive and @f$V_0@f$ is low.
  * In case of Pull down configuration @f$\frac{dV}{dT}@f$ is negative and @f$V_0@f$ is high.
  *
  * In case a real temperature sensor is not available (Sensor Type = #VIRTUAL_SENSOR),
  * This component will always returns a constant, programmable, temperature.
  *
  * @{
  */

/* Private function prototypes -----------------------------------------------*/
uint16_t NTC_SetFaultState(NTC_Handle_t *pHandle);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief Returns fault when temperature exceeds the related temperature voltage protection threshold
  *
  * @param pHandle : Pointer on Handle structure of TemperatureSensor component
  *
  * @retval Fault status : Updated internal fault status
  */
__weak uint16_t NTC_SetFaultState(NTC_Handle_t *pHandle)
{
  uint16_t hFault;
#ifdef NULL_PTR_CHECK_NTC_TEMP_SENS
  if (MC_NULL == pHandle)
  {
    hFault = MC_SW_ERROR;
  }
  else
  {
#endif
    if (pHandle->hAvTemp_d > pHandle->hOverTempThreshold)
    {
      hFault = MC_OVER_TEMP;
    }
    else if (pHandle->hAvTemp_d < pHandle->hOverTempDeactThreshold)
    {
      hFault = MC_NO_ERROR;
    }
    else
    {
      hFault = pHandle->hFaultState;
    }
#ifdef NULL_PTR_CHECK_NTC_TEMP_SENS
  }
#endif
  return hFault;
}

/* Functions ---------------------------------------------------- */

/**
  * @brief Initializes temperature sensing conversions
  *
  * @param pHandle : Pointer on Handle structure of TemperatureSensor component
  *
  */
__weak void NTC_Init(NTC_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_NTC_TEMP_SENS
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    if (REAL_SENSOR == pHandle->bSensorType)
    {
      /* Need to be register with RegularConvManager */
      pHandle->convHandle = RCM_RegisterRegConv(&pHandle->TempRegConv);
      NTC_Clear(pHandle);
    }
    else  /* case VIRTUAL_SENSOR */
    {
      pHandle->hFaultState = MC_NO_ERROR;
      pHandle->hAvTemp_d = pHandle->hExpectedTemp_d;
    }
#ifdef NULL_PTR_CHECK_NTC_TEMP_SENS
  }
#endif
}

/**
  * @brief Initializes internal average temperature computed value
  *
  * @param pHandle : Pointer on Handle structure of TemperatureSensor component
  */
__weak void NTC_Clear(NTC_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_NTC_TEMP_SENS
  if (MC_NULL == pHandle)
  {
    /* nothing to do */
  }
  else
  {
#endif
    pHandle->hAvTemp_d = 0U;
#ifdef NULL_PTR_CHECK_NTC_TEMP_SENS
  }
#endif
}

/**
  * @brief Performs the temperature sensing average computation after an ADC conversion
  *
  * @param pHandle : Pointer on Handle structure of TemperatureSensor component
  *
  * @retval Fault status : Error reported in case of an over temperature detection
  */
__weak uint16_t NTC_CalcAvTemp(NTC_Handle_t *pHandle)
{
  uint16_t returnValue;
#ifdef NULL_PTR_CHECK_NTC_TEMP_SENS
  if (MC_NULL == pHandle)
  {
    returnValue = 0U;
  }
  else
  {
#endif
    if (REAL_SENSOR == pHandle->bSensorType)
    {
      uint16_t hAux;
      hAux = RCM_ExecRegularConv(pHandle->convHandle);

      if (0xFFFFU == hAux)
      {
        /* Nothing to do */
      }
      else
      {
        uint32_t wtemp;
        wtemp = (uint32_t)(pHandle->hLowPassFilterBW) - 1U;
        wtemp *= ((uint32_t)pHandle->hAvTemp_d);
        wtemp += hAux;
        wtemp /= ((uint32_t)pHandle->hLowPassFilterBW);

        pHandle->hAvTemp_d = (uint16_t)wtemp;
      }

      pHandle->hFaultState = NTC_SetFaultState(pHandle);
    }
    else  /* case VIRTUAL_SENSOR */
    {
      pHandle->hFaultState = MC_NO_ERROR;
    }
    returnValue = pHandle->hFaultState;
#ifdef NULL_PTR_CHECK_NTC_TEMP_SENS
  }
#endif
  return (returnValue);
}

/**
  * @brief  Returns latest averaged temperature measured expressed in u16Celsius
  *
  * @param pHandle : Pointer on Handle structure of TemperatureSensor component
  *
  * @retval AverageTemperature : Current averaged temperature measured (in u16Celsius)
  */
__weak uint16_t NTC_GetAvTemp_d(const NTC_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_NTC_TEMP_SENS
  return ((MC_NULL == pHandle) ? 0U : pHandle->hAvTemp_d);
#else
  return (pHandle->hAvTemp_d);
#endif
}

/**
  * @brief  Returns latest averaged temperature expressed in Celsius degrees
  *
  * @param pHandle : Pointer on Handle structure of TemperatureSensor component
  *
  * @retval AverageTemperature : Latest averaged temperature measured (in Celsius degrees)
  */
__weak int16_t NTC_GetAvTemp_C(NTC_Handle_t *pHandle)
{
  int16_t returnValue;
#ifdef NULL_PTR_CHECK_NTC_TEMP_SENS
  if (MC_NULL == pHandle)
  {
    returnValue = 0;
  }
  else
  {
#endif
    int32_t wTemp;

    if (REAL_SENSOR == pHandle->bSensorType)
    {
      wTemp = (int32_t)pHandle->hAvTemp_d;
      wTemp -= ((int32_t)pHandle->wV0);
      wTemp *= pHandle->hSensitivity;
#ifndef FULL_MISRA_C_COMPLIANCY_NTC_TEMP
      //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
      wTemp = (wTemp >> 16) + (int32_t)pHandle->hT0;
#else
      wTemp = (wTemp / 65536) + (int32_t)pHandle->hT0;
#endif
    }
    else
    {
      wTemp = (int32_t)pHandle->hExpectedTemp_C;
    }
    returnValue = (int16_t)wTemp;
#ifdef NULL_PTR_CHECK_NTC_TEMP_SENS
  }
#endif
  return (returnValue);
}

/**
  * @brief  Returns Temperature measurement fault status
  *
  * Fault status can be either MC_OVER_TEMP when measure exceeds the protection threshold values or
  * MC_NO_ERROR if it is inside authorized range.
  *
  * @param pHandle: Pointer on Handle structure of TemperatureSensor component.
  *
  * @retval Fault status : read internal fault state
  */
__weak uint16_t NTC_CheckTemp(const NTC_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_NTC_TEMP_SENS
  return ((MC_NULL == pHandle) ? 0U : pHandle->hFaultState);
#else
  return (pHandle->hFaultState);
#endif

}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
