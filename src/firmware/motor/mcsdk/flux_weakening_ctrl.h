/**
  ******************************************************************************
  * @file    flux_weakening_ctrl.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides all definitions and functions prototypes for the 
  *          Flux Weakening Control component of the Motor Control SDK.
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
  * @ingroup FluxWeakeningCtrl
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef FLUXWEAKENINGCTRL_H
#define FLUXWEAKENINGCTRL_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "pid_regulator.h"
/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup FluxWeakeningCtrl
  * @{
  */

/**
  * @brief  Flux Weakening Control Component handle structure
  */
typedef struct
{
  PID_Handle_t   *pFluxWeakeningPID;      /**< @brief PI object used for flux weakening */
  PID_Handle_t   *pSpeedPID;              /**< @brief PI object used for speed control */
  uint16_t        hFW_V_Ref;              /**< @brief Voltage reference, 
                                                expressed in tenth ofpercentage points */
  qd_t            AvVolt_qd;              /**< @brief Average stator voltage in qd
                                                 reference frame */
  int16_t         AvVoltAmpl;             /**< @brief Average stator voltage amplitude */
  int16_t         hIdRefOffset;           /**< @brief Id reference offset */
  uint16_t        hMaxModule;             /**< @brief Circle limitation maximum allowed module */

  uint16_t        hDefaultFW_V_Ref;       /**< @brief Default flux weakening voltage reference,
                                               tenth of percentage points*/
  int16_t         hDemagCurrent;          /**< @brief Demagnetization current in s16A:
                                               Current(Amp) = [Current(s16A) * Vdd micro]/
                                               [65536 * Rshunt * Aop] */
  int32_t         wNominalSqCurr;         /**< @brief Squared motor nominal current in (s16A)^2
                                               where:
                                               Current(Amp) = [Current(s16A) * Vdd micro]/
                                               [65536 * Rshunt * Aop] */
  uint16_t        hVqdLowPassFilterBW;    /**< @brief Use this parameter to configure the Vqd
                                               first order software filter bandwidth.
                                               hVqdLowPassFilterBW = FOC_CurrController
                                               call rate [Hz]/ FilterBandwidth[Hz] in
                                               case FULL_MISRA_COMPLIANCY is defined.
                                               On the contrary, if FULL_MISRA_COMPLIANCY
                                               is not defined, hVqdLowPassFilterBW is
                                               equal to log with base two of previous
                                               definition */
  uint16_t        hVqdLowPassFilterBWLOG; /**< @brief hVqdLowPassFilterBW expressed as power of 2.
                                               E.g. if gain divisor is 512 the value
                                               must be 9 because 2^9 = 512 */
} FW_Handle_t;

/* Exported functions ------------------------------------------------------- */

/**
  * Initializes flux weakening component handler.
  */
void FW_Init(FW_Handle_t *pHandle, PID_Handle_t *pPIDSpeed, PID_Handle_t *pPIDFluxWeakeningHandle);

/**
  * Clears the flux weakening internal variables
  */
void FW_Clear(FW_Handle_t *pHandle);

/**
  * Computes Iqdref according the flux weakening algorithm.  
  */
qd_t FW_CalcCurrRef(FW_Handle_t *pHandle, qd_t Iqdref);

/**
  * Applies a low-pass filter on both  Vqd voltage components.
  */
void FW_DataProcess(FW_Handle_t *pHandle, qd_t Vqd);

/**
  * Sets a new value for the voltage reference used by
  * flux weakening algorithm.
  */
void FW_SetVref(FW_Handle_t *pHandle, uint16_t hNewVref);

/**
  * Returns the present value of target voltage used by flux
  * weakening algorihtm.
  */
uint16_t FW_GetVref(FW_Handle_t *pHandle);

/**
  * Returns the present value of voltage actually used by flux
  * weakening algorihtm.
  */
int16_t FW_GetAvVAmplitude(FW_Handle_t *pHandle);

/**
  * Returns the measure of present voltage actually used by flux
  * weakening algorihtm as percentage of available voltage.
  */
uint16_t FW_GetAvVPercentage(FW_Handle_t *pHandle);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* FLUXWEAKENINGCTRL_H */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
