/**
  ******************************************************************************
  * @file    pqd_motor_power_measurement.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          PQD Motor Power Measurement component of the Motor Control SDK.
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
  * @ingroup pqd_motorpowermeasurement
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PQD_MOTORPOWERMEASUREMENT_H
#define PQD_MOTORPOWERMEASUREMENT_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "bus_voltage_sensor.h"


/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pqd_motorpowermeasurement
  * @{
  */

/**
 * @brief Handle of a PQD Motor Power Measurement component
 * 
 * PQD Motor Power Measurement components compute a value of the average electrical power 
 * flowing into a motor from the QD-frame @f$I_{qd}@f$ and @f$V_{qd}@f$ values.
 * 
 */
typedef struct
{
  int16_t hAvrgElMotorPower;  /**< @brief Average measured motor power expressed in s16 digit (s16A x s16V). */

  float ConvFact;             /**< @brief Factor used to convert s16 digit average motor power values into Watts. 
                                          Set to @f[\sqrt{3} \frac{V_{dd}}{R_{shunt} \times A_{op} \times 65536}@f] */

  pFOCVars_t pFOCVars;               /**< @brief Pointer to FOC vars. */
  BusVoltageSensor_Handle_t *pVBS;   /**< @brief Bus voltage sensor component handle. */
} PQD_MotorPowMeas_Handle_t;


/* Updates the average electrical motor power measure with the latest values of the DQ currents and voltages. */
void PQD_CalcElMotorPower(PQD_MotorPowMeas_Handle_t *pHandle);

/* Clears the the int16_t digit average motor power value stored in the handle */
void PQD_Clear(PQD_MotorPowMeas_Handle_t *pHandle);

/* Returns an average value of the measured motor power expressed in Watts */
float PQD_GetAvrgElMotorPowerW(const PQD_MotorPowMeas_Handle_t *pHandle);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* PQD_MOTORPOWERMEASUREMENT_H */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
