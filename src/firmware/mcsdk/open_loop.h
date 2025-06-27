/**
  ******************************************************************************
  * @file    open_loop.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          Open Loop component of the Motor Control SDK.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef OPENLOOPCLASS_H
#define OPENLOOPCLASS_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "virtual_speed_sensor.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup OpenLoop
  * @{
  */

/**
  * @brief  OpenLoop_Handle_t structure used for phases definition
  */
typedef struct
{
  int16_t hDefaultVoltage; /**< @brief Default Open loop phase voltage. */

  bool VFMode;             /**< @brief Flag to enable Voltage versus Frequency mode (V/F mode). */

  int16_t hVFOffset;       /**< @brief Minimum Voltage to apply when frequency is equal to zero. */

  int16_t hVFSlope;        /**< @brief Slope of V/F curve: Voltage = (hVFSlope)*Frequency + hVFOffset. */

  int16_t hVoltage;        /**< @brief Current Open loop phase voltage. */

  VirtualSpeedSensor_Handle_t *pVSS; /**< @brief Allow access on mechanical speed measured. */

} OpenLoop_Handle_t;

/* Exported functions ------------------------------------------------------- */

/* Initializes OpenLoop variables. */
void OL_Init(OpenLoop_Handle_t *pHandle, VirtualSpeedSensor_Handle_t *pVSS);

/* Sets Vqd according to open loop phase voltage.  */
qd_t OL_VqdConditioning(const OpenLoop_Handle_t *pHandle);

/* Sets new open loop phase voltage */
void OL_UpdateVoltage(OpenLoop_Handle_t *pHandle, int16_t hNewVoltage);

/* Gets open loop phase voltage. */
int16_t OL_GetVoltage( OpenLoop_Handle_t * pHandle );

/* Computes phase voltage to apply according to average mechanical speed (V/F Mode). */
void OL_Calc(OpenLoop_Handle_t *pHandle);

/* Activates of the Voltage versus Frequency mode (V/F mode) */
void OL_VF(OpenLoop_Handle_t *pHandle, bool VFEnabling);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* OPENLOOPCLASS_H */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
