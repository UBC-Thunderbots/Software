/**
  ******************************************************************************
  * @file    feed_forward_ctrl.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          Feed Forward Control component of the Motor Control SDK.
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
  * @ingroup FeedForwardCtrl
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef FEEDFORWARDCTRL_H
#define FEEDFORWARDCTRL_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "bus_voltage_sensor.h"
#include "speed_pos_fdbk.h"
#include "speed_torq_ctrl.h"


/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup FeedForwardCtrl
  * @{
  */

/**
  * @brief Handle structure of the Feed-forward Component
  */
typedef struct
{
  qd_t   Vqdff;                 /**< Feed Forward controller @f$I_{qd}@f$ contribution to @f$V_{qd}@f$ */
  qd_t   VqdPIout;              /**< @f$V_{qd}@f$ as output by PID controller */
  qd_t   VqdAvPIout;            /**< Averaged @f$V_{qd}@f$ as output by PID controller */
  int32_t  wConstant_1D;                   /**< Feed forward default constant for the @f$d@f$ axis */
  int32_t  wConstant_1Q;                   /**< Feed forward default constant for the @f$q@f$ axis */
  int32_t  wConstant_2;                    /**< Default constant value used by Feed-forward algorithm */
  BusVoltageSensor_Handle_t *pBus_Sensor;  /**< Related bus voltage sensor */
  PID_Handle_t *pPID_q;                    /*!< Related PI for @f$I_{q}@f$ regulator */
  PID_Handle_t *pPID_d;                    /*!< Related PI for @f$I_{d}@f$ regulator */
  uint16_t hVqdLowPassFilterBW;            /**< Use this parameter to configure the Vqd
                                               first order software filter bandwidth. In
                                               case FULL_MISRA_COMPLIANCY,
                                               hVqdLowPassFilterBW is used and equal to
                                               FOC_CurrController call rate [Hz]/ FilterBandwidth[Hz].
                                               On the contrary, if FULL_MISRA_COMPLIANCY
                                               is not defined, hVqdLowPassFilterBWLOG is
                                               used and equal to log with base two of previous
                                               definition */
  int32_t  wDefConstant_1D;                /**< Feed forward default constant for d axes */
  int32_t  wDefConstant_1Q;                /**< Feed forward default constant for q axes */
  int32_t  wDefConstant_2;                 /**< Default constant value used by
                                                Feed-forward algorithm*/
  uint16_t hVqdLowPassFilterBWLOG;         /**< hVqdLowPassFilterBW expressed as power of 2.
                                                E.g. if gain divisor is 512 the value
                                                must be 9 because 2^9 = 512 */

} FF_Handle_t;





/*
 * Initializes all the component variables
 */
void FF_Init(FF_Handle_t *pHandle, BusVoltageSensor_Handle_t *pBusSensor, PID_Handle_t *pPIDId,
             PID_Handle_t *pPIDIq);

/*
 * It should be called before each motor restart and clears the Feed-forward
 * internal variables.
 */
void FF_Clear(FF_Handle_t *pHandle);

/*
 * It implements Feed-forward controller by computing new Vqdff value.
 * This will be then summed up to PI output in FF_VqdConditioning
 * method.
 */
void FF_VqdffComputation(FF_Handle_t *pHandle, qd_t Iqdref, SpeednTorqCtrl_Handle_t *pSTC);

/*
 * It returns the Vqd components computed by input plus the Feed-forward
 * action and store the last Vqd values in the internal variable.
 */
qd_t FF_VqdConditioning(FF_Handle_t *pHandle, qd_t Vqd);

/*
 * It low-pass filters the Vqd voltage coming from the speed PI. Filter
 * bandwidth depends on hVqdLowPassFilterBW parameter.
 */
void FF_DataProcess(FF_Handle_t *pHandle);

/*
 * Use this method to initialize FF variables in START_TO_RUN state.
 */
void FF_InitFOCAdditionalMethods(FF_Handle_t *pHandle);

/*
 *  Use this method to set new constants values used by
 *  Feed-forward algorithm.
 */
void FF_SetFFConstants(FF_Handle_t *pHandle, FF_TuningStruct_t sNewConstants);

/*
 * Use this method to get present constants values used by
 * Feed-forward algorithm.
 */
FF_TuningStruct_t FF_GetFFConstants(FF_Handle_t *pHandle);

/*
 * Use this method to get present values for the Vqd Feed-forward
 * components.
 */
qd_t FF_GetVqdff(const FF_Handle_t *pHandle);

/*
 * Use this method to get the averaged output values of qd axes
 * currents PI regulators.
 */
qd_t FF_GetVqdAvPIout(const FF_Handle_t *pHandle);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* FEEDFORWARDCTRL_H */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
