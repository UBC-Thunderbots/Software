/**
  ******************************************************************************
  * @file    bus_voltage_sensor.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the BusVoltageSensor component of the Motor Control SDK.
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
  * @ingroup BusVoltageSensor
  */

/* Includes ------------------------------------------------------------------*/

#include "bus_voltage_sensor.h"


/** @addtogroup MCSDK
  * @{
  */

/** @defgroup BusVoltageSensor Bus Voltage Sensing
  * @brief Bus Voltage Sensor components of the Motor Control SDK
  *
  * Two Bus Voltage Sensor implementations are provided (selection done according to BUS_VOLTAGE_READING definition):
  *
  * - The @ref RDividerBusVoltageSensor "Resistor Divider Bus Voltage Sensor" operates as the name suggests
  * - The @ref VirtualBusVoltageSensor "Virtual Bus Voltage Sensor" does not make measurement but rather
  *   returns an application fixed defined value (expected VBus value).
  *
  *   Two formats are used to return VBus measurement:
  *   - Volt
  *   - u16Volt that represents the ADC converted signal measuring the voltage at sensing network attenuation output
  * @f[
  * u16Volt = (ADC\_REFERENCE\_VOLTAGE / VBUS\_PARTITIONING\_FACTOR) / 65536
  * @f]
  *
  * @{
  */

/**
  * @brief  It return latest Vbus conversion result expressed in u16Volt format
  * @param  pHandle related Handle of BusVoltageSensor_Handle_t
  * @retval uint16_t Latest Vbus conversion result in u16Volt format
  */
__weak uint16_t VBS_GetBusVoltage_d(const BusVoltageSensor_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_BUS_VOLT
  uint16_t temp_latestConv;
  if (MC_NULL == pHandle)
  {
    temp_latestConv = 0;
  }
  else
  {
    temp_latestConv = pHandle->LatestConv;
  }
  return (temp_latestConv);
#else
  return (pHandle->LatestConv);
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
  * @brief  It return latest averaged Vbus measurement expressed in u16Volt format
  * @param  pHandle related Handle of BusVoltageSensor_Handle_t
  * @retval uint16_t Latest averaged Vbus measurement in u16Volt format
  */
__weak uint16_t VBS_GetAvBusVoltage_d(const BusVoltageSensor_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_BUS_VOLT
  uint16_t temp_avBusVoltage_d;

  if (MC_NULL == pHandle)
  {
    temp_avBusVoltage_d = 0U;
  }
  else
  {
    temp_avBusVoltage_d = pHandle->AvBusVoltage_d;
  }
  return (temp_avBusVoltage_d);
#else
  return (pHandle->AvBusVoltage_d);
#endif
}


/**
  * @brief  It return latest averaged Vbus measurement expressed in Volt format
  * @param  pHandle related Handle of BusVoltageSensor_Handle_t
  * @retval uint16_t Latest averaged Vbus measurement in Volt format
  */
__weak uint16_t VBS_GetAvBusVoltage_V(const BusVoltageSensor_Handle_t *pHandle)
{
  uint32_t temp;
#ifdef NULL_PTR_CHECK_BUS_VOLT
  if (MC_NULL == pHandle)
  {
    temp = 0U;
  }
  else
  {
#endif
    temp = (uint32_t)(pHandle->AvBusVoltage_d);
    temp *= pHandle->ConversionFactor;
    temp /= 65536U;
#ifdef NULL_PTR_CHECK_BUS_VOLT
  }
#endif
  return ((uint16_t)temp);
}

/**
  * @brief  It returns MC_OVER_VOLT, MC_UNDER_VOLT or MC_NO_ERROR depending on
  *         bus voltage and protection threshold values
  * @param  pHandle related Handle of BusVoltageSensor_Handle_t
  * @retval uint16_t Fault code error
  */
__weak uint16_t VBS_CheckVbus(const BusVoltageSensor_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_BUS_VOLT
  uint16_t temp_faultState;

  if (MC_NULL ==  pHandle)
  {
    temp_faultState = 0U;
  }
  else
  {
    temp_faultState = pHandle->FaultState;
  }
  return (temp_faultState);
#else
  return (pHandle->FaultState);
#endif
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
