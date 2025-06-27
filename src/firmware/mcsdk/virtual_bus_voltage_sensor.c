/**
  ******************************************************************************
  * @file    virtual_bus_voltage_sensor.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the Virtual Bus Voltage Sensor component of the Motor Control SDK.
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
  * @ingroup VirtualBusVoltageSensor
  */

/* Includes ------------------------------------------------------------------*/
#include "virtual_bus_voltage_sensor.h"


/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup BusVoltageSensor
  * @{
  */

/** @defgroup VirtualBusVoltageSensor Virtual Bus Voltage Sensor
  * @brief Virtual Bus Voltage Sensor implementation.
  *
  * @{
  */

/**
  * @brief  It initializes bus voltage conversion for virtual bus voltage sensor
  *         (latest value and averaged value) with expected VBus value
  * @param  pHandle related Handle of VirtualBusVoltageSensor_Handle_t
  * @retval none
  */
__weak void VVBS_Init(VirtualBusVoltageSensor_Handle_t *pHandle)
{
  pHandle->_Super.FaultState = MC_NO_ERROR;
  pHandle->_Super.LatestConv = pHandle->ExpectedVbus_d;
  pHandle->_Super.AvBusVoltage_d = pHandle->ExpectedVbus_d;
}

/**
  * @brief  Empty function (no process and no returned value)
  * @param  pHandle related Handle of VirtualBusVoltageSensor_Handle_t
  * @retval none
  */
__weak void VVBS_Clear(VirtualBusVoltageSensor_Handle_t *pHandle)
{
  return;
}

/**
  * @brief  It returns MC_NO_ERROR
  * @param  pHandle related Handle of VirtualBusVoltageSensor_Handle_t
  * @retval uint16_t Fault code error: MC_NO_ERROR
  */
__weak uint16_t VVBS_NoErrors(VirtualBusVoltageSensor_Handle_t *pHandle)
{
  return (MC_NO_ERROR);
}

/**
  * @}
  */

/**
  * @}
  */

/** @} */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/

