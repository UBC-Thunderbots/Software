/**
  ******************************************************************************
  * @file    speed_pos_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides all definitions and functions prototypes
  *          of the Speed & Position Feedback component of the Motor Control SDK.
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
  * @ingroup SpeednPosFdbk
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SPEEDNPOSFDBK_H
#define SPEEDNPOSFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
/* Already into mc_type.h */
/* #include "stdint.h" */
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/**
  * @brief  SpeednPosFdbk  handles definitions of mechanical and electrical speed, mechanical acceleration, mechanical and electrical angle and all
  *                        constants and scale values for a reliable measure and computation in appropriated unit.
  */
typedef struct
{

  uint8_t bSpeedErrorNumber;          /*!< Number of time the average mechanical speed is not valid. */
  uint8_t bElToMecRatio;              /*!< Coefficient used to transform electrical to mechanical quantities and viceversa.
                                           It usually coincides with motor pole pairs number. */
  uint8_t SpeedUnit;                  /*!< The speed unit value is defined into mc_stm_types.h by [SPEED_UNIT](measurement_units.md) in tenth of Hertz.*/
  uint8_t bMaximumSpeedErrorsNumber;  /*!< Maximum value of not valid speed measurements before an error is reported.*/
  int16_t hElAngle;                   /*!< Estimated electrical angle reported by the implemented speed and position method. */
  int16_t hMecAngle;                  /*!< Instantaneous measure of rotor mechanical angle. */
  int32_t wMecAngle;                  /*!< Mechanical angle frame based on coefficient #bElToMecRatio. */
  int16_t hAvrMecSpeedUnit;           /*!< Average mechanical speed expressed in the unit defined by [SPEED_UNIT](measurement_units.md). */
  int16_t hElSpeedDpp;                /*!< Instantaneous electrical speed expressed in Digit Per control Period ([dpp](measurement_units.md)), expresses
                                           the angular speed as the variation of the electrical angle. */
  int16_t InstantaneousElSpeedDpp;    /*!< Instantaneous computed electrical speed, expressed in [dpp](measurement_units.md). */
  int16_t hMecAccelUnitP;             /*!< Average mechanical acceleration expressed in the unit defined by #SpeedUnit,
                                           only reported with encoder implementation */
  uint16_t hMaxReliableMecSpeedUnit;  /*!< Maximum value of measured mechanical speed that is considered to be valid.
                                           Expressed in the unit defined by [SPEED_UNIT](measurement_units.md). */
  uint16_t hMinReliableMecSpeedUnit;  /*!< Minimum value of measured mechanical speed that is considered to be valid.
                                           Expressed in the unit defined by [SPEED_UNIT](measurement_units.md).*/
  uint16_t hMaxReliableMecAccelUnitP; /*!< Maximum value of measured acceleration that is considered to be valid.
                                           Constant value equal to 65535, expressed in the unit defined by [SPEED_UNIT](measurement_units.md). */
  uint16_t hMeasurementFrequency;     /*!< Frequency at which the user will request a measurement of the rotor electrical
                                           angle. Expressed in PWM_FREQ_SCALING * Hz. */
  uint32_t DPPConvFactor;             /*!< Conversion factor (65536/#PWM_FREQ_SCALING) used to convert measured speed
                                           from the unit defined by [SPEED_UNIT](measurement_units.md) to [dpp](measurement_units.md). */


} SpeednPosFdbk_Handle_t;

/**
  * @brief input structure type definition for SPD_CalcAngle
  */
typedef struct
{
  alphabeta_t  Valfa_beta;            /*!< Voltage Components in alfa beta reference frame */
  alphabeta_t  Ialfa_beta;            /*!< Current Components in alfa beta reference frame */
  uint16_t     Vbus;                  /*!< Virtual Bus Voltage information */
} Observer_Inputs_t;


int16_t SPD_GetElAngle(const SpeednPosFdbk_Handle_t *pHandle);

int32_t SPD_GetMecAngle(const SpeednPosFdbk_Handle_t *pHandle);

int16_t SPD_GetAvrgMecSpeedUnit(const SpeednPosFdbk_Handle_t *pHandle);

int16_t SPD_GetElSpeedDpp(const SpeednPosFdbk_Handle_t *pHandle);

int16_t SPD_GetInstElSpeedDpp(const SpeednPosFdbk_Handle_t *pHandle);

bool SPD_Check(const SpeednPosFdbk_Handle_t *pHandle);

bool SPD_IsMecSpeedReliable(SpeednPosFdbk_Handle_t *pHandle, const int16_t *pMecSpeedUnit);

int16_t SPD_GetS16Speed(const SpeednPosFdbk_Handle_t *pHandle);

uint8_t SPD_GetElToMecRatio(const SpeednPosFdbk_Handle_t *pHandle);

void SPD_SetElToMecRatio(SpeednPosFdbk_Handle_t *pHandle, uint8_t bPP);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* SPEEDNPOSFDBK_H */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
