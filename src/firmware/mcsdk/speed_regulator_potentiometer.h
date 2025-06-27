/**
  ******************************************************************************
  * @file    speed_regulator_potentiometer.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          Speed Regulator Potentiometer component of the Motor Control SDK.
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
  * @ingroup SpeedRegulatorPotentiometer
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SPEEDREGULATOR_H
#define SPEEDREGULATOR_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "speed_torq_ctrl.h"
#include "regular_conversion_manager.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeedRegulatorPotentiometer
  * @{
  */


/**
  * @brief structure used for external speed regulator
  *
  */
typedef struct
{
  RegConv_t SpeedRegConv;
  SpeednTorqCtrl_Handle_t *pSTC;  /**< Speed and torque controller object used by the Speed Regulator.*/
  uint16_t ConversionFactor;      /**< Factor to convert speed between u16digit and #SPEED_UNIT. */
  uint16_t LatestConv;            /**< Contains the latest potentiometer converted value expressed 
                                       in u16digit format */
  uint16_t AvSpeedReg_d;          /**< Contains the latest available average speed expressed in 
                                       u16digit */
  //uint16_t 

  uint16_t LowPassFilterBW;       /**< Used to configure the first order software filter bandwidth.
                                       hLowPassFilterBW = SRP_CalcSpeedReading
                                       call rate [Hz]/ FilterBandwidth[Hz] */
  uint16_t SpeedAdjustmentRange;  /**< Represents the range used to trigger a speed ramp command*/
  uint16_t MinimumSpeedRange;     /**< Minimum settable speed expressed in u16digit*/
  uint16_t MaximumSpeedRange;     /**< Maximum settable speed expressed in u16digit*/
  uint32_t RampSlope;             /**< Speed variation in SPPED_UNIT/s. */
  uint16_t *aBuffer;              /**< Buffer used to compute average value.*/
  uint8_t  elem;                  /**< Number of stored elements in the average buffer.*/
  uint8_t  index;                 /**< Index of last stored element in the average buffer.*/
  uint8_t convHandle;             /**< Handle on the regular conversion */
  bool OutOfSynchro;              /**< Check of the matching between potentiometer setting and measured speed */
} SRP_Handle_t;

/* Initializes a Speed Regulator Potentiometer component */
void SRP_Init(SRP_Handle_t *pHandle, SpeednTorqCtrl_Handle_t *pSTC);
/* Clears a Speed Regulator Potentiometer component */
void SRP_Clear( SRP_Handle_t * pHandle );
/* Reads the potentiometer of an SRP component to compute an average speed reference */
bool SRP_CalcAvSpeedReg( SRP_Handle_t * pHandle );
/* Reads the potentiometer of an SRP component and filters it to compute an average speed reference */
bool SRP_CalcAvSpeedRegFilt( SRP_Handle_t * pHandle );
bool SRP_ExecPotRamp( SRP_Handle_t * pHandle );
bool SRP_CheckSpeedRegSync( SRP_Handle_t * pHandle );
uint16_t SRP_GetSpeedReg_d( SRP_Handle_t * pHandle );
uint16_t SRP_GetAvSpeedReg_d( SRP_Handle_t * pHandle );
uint16_t SRP_GetAvSpeedReg_SU( SRP_Handle_t * pHandle );

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* SPEEDREGULATOR_H */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
