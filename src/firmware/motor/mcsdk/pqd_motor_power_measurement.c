/**
  ******************************************************************************
  * @file    pqd_motor_power_measurement.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides the functions that implement the  features of the 
  *          PQD Motor Power Measurement component of the Motor Control SDK.
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
  * @ingroup pqd_motorpowermeasurement
  */

/* Includes ------------------------------------------------------------------*/

#include "pqd_motor_power_measurement.h"
#include "mc_type.h"


/** @addtogroup MCSDK
  * @{
  */

/** @defgroup pqd_motorpowermeasurement PQD Motor Power Measurement
  * @brief Motor Power Measurement component using DQ-frame current and voltage values 
  *
  * The PQD Motor Power Measurement component uses @f$I_d@f$, @f$I_q@f$, @f$V_d@f$ and @f$V_q@f$
  * to compute the electrical power flowing through the motor.
  * 
  * These values are periodically sampled from the current regulation loop and used to compute 
  * instantaneous power values. The instantaneous values are then used to compute an average
  * power value that is stored. These computations are done with integer operations and the
  * average value is store as an integer, in s16 digit format (s16A x s16V unit). 
  * 
  * The PQD Motor Power Measurement component provides an interface, PQD_GetAvrgElMotorPowerW()
  * that converts the int16_t digit average power into a floating point value expressed in
  * Watts.
  *
  * @{
  */

/**
  * @brief Updates the average electrical motor power measure with the latest values 
  *        of the DQ currents and voltages.
  * 
  * This method should be called with Medium Frequency Task periodicity. It computes an
  * instantaneous power value using the latest @f$I_{qd}@f$ and @f$V_{qd}@f$ data available
  * and uses it to update the average motor power value. 
  * 
  * Computations are done on s16A and s16V integer values defined in
  * [Current measurement unit](measurement_units.md). The average motor power value is
  * computed as an int16_t value.
  * 
  * @param pHandle Handle on the related PQD Motor Power Measurement component instance.
  */
__weak void PQD_CalcElMotorPower(PQD_MotorPowMeas_Handle_t *pHandle)
{
#ifdef NULL_PTR_MOT_POW_MEAS
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    int32_t wAux;
    qd_t Iqd = pHandle->pFOCVars->Iqd;
    qd_t Vqd = pHandle->pFOCVars->Vqd;

    wAux = ((int32_t)Iqd.q * (int32_t)Vqd.q)
         + ((int32_t)Iqd.d * (int32_t)Vqd.d);
    wAux /= 65536;

    pHandle->hAvrgElMotorPower += (wAux - pHandle->hAvrgElMotorPower) >> 4;

#ifdef NULL_PTR_MOT_POW_MEAS
  }
#endif
}

/**
  * @brief  Clears the int16_t digit average motor power value stored in the handle.
  * 
  * This function should be called before each motor start.
  * 
  * @param pHandle Handle on the related PQD Motor Power Measurement component instance.
  */
__weak void PQD_Clear(PQD_MotorPowMeas_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_MOT_POW_MES
  if (MC_NULL == pHandle)
  {
    /* nothing to do */
  }
  else
  {
#endif
    pHandle->hAvrgElMotorPower = 0;
#ifdef NULL_PTR_CHECK_MOT_POW_MES
  }
#endif
}

/**
  * @brief Returns an average value of the measured motor power expressed in Watts
  * 
  * This function converts the int16_t digit average motor power value stored in the handle
  * in a floating point value in Watts.
  * 
  * @param pHandle pointer on the related component instance.
  * @retval float The average measured motor power expressed in Watts.
  */
__weak float PQD_GetAvrgElMotorPowerW(const PQD_MotorPowMeas_Handle_t *pHandle)
{
  float PowerW=0.0;
  
#ifdef NULL_PTR_MOT_POW_MEAS
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif

  /* First perform an integer multiplication, then a float one. */
  PowerW = (pHandle->hAvrgElMotorPower * VBS_GetAvBusVoltage_V(pHandle->pVBS)) * pHandle->ConvFact;

#ifdef NULL_PTR_MOT_POW_MEAS
  }
#endif
  return (PowerW);
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
