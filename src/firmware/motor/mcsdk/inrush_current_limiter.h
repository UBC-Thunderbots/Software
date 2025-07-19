/**
  ******************************************************************************
  * @file    inrush_current_limiter.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          Inrush Current Limiter component featuring the Motor Control SDK.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __INRUSHCURRENTLIMITER_H
#define __INRUSHCURRENTLIMITER_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "bus_voltage_sensor.h"
#include "digital_output.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup ICL
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/**
  * @brief ICL_State_t defines all the existing ICL states of the state machine
  */
typedef enum
{
  ICL_IDLE,             /**< @brief stable state */
  ICL_ACTIVATION,       /**< @brief transition state */
  ICL_ACTIVE,           /**< @brief stable state */
  ICL_DEACTIVATION,     /**< @brief transition state */
  ICL_INACTIVE          /**< @brief stable state */
} ICL_State_t;


/**
  * @brief  ICL_Handle_t is used to handle an instance of the InrushCurrentLimiter component
  */
typedef struct
{
  BusVoltageSensor_Handle_t *pVBS;  /**< @brief Vbus handler used for the automatic ICL component activation/deactivation */
  DOUT_handle_t *pDOUT;             /**< @brief digital output handler used to physically activate/deactivate the ICL component */
  ICL_State_t ICLstate;             /**< @brief Current state of the ICL state machine */
  uint16_t hICLTicksCounter;        /**< @brief Buffer variable containing the number of clock events remaining for next state transition */
  uint16_t hICLSwitchDelayTicks;    /**< @brief ICL activation/deactivation delay due to switching action of relay (expressed in ICL FSM execution ticks) */
  uint16_t hICLChargingDelayTicks;  /**< @brief Input capacitors charging delay to be waited before closing relay (expressed in ICL FSM execution ticks)*/
  uint16_t hICLVoltageThreshold;    /**< @brief Voltage threshold to be reached on Vbus before closing the relay */
} ICL_Handle_t;


/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/* Initializes all the needed ICL component variables */
void ICL_Init(ICL_Handle_t *pHandle, BusVoltageSensor_Handle_t *pVBS, DOUT_handle_t *pDOUT);

/* Executes the Inrush Current Limiter state machine */
ICL_State_t ICL_Exec(ICL_Handle_t *pHandle);

/* Returns the current state of the ICL state machine. */
ICL_State_t ICL_GetState(ICL_Handle_t *pHandle);

/**
  * @brief  Converts the VoltageThreshold configured by the user and updates the 
  *         passed ICL's component threshold value in u16V (bus sensor units)
  * @param  pHandle: handler of the current instance of the ICL component
  * @param  hVoltageThreshold : threshold configured by the user to be applied in ICL FSM (expressed in Volts) 
  */
inline static void ICL_SetVoltageThreshold(ICL_Handle_t *pHandle, int16_t hVoltageThreshold)
{
  uint32_t temp;

  temp = (uint32_t)hVoltageThreshold;
  temp *= 65536U;
  temp /= pHandle->pVBS->ConversionFactor;

  pHandle->hICLVoltageThreshold = (uint16_t)temp;
}

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __INRUSHCURRENTLIMITER_H */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
