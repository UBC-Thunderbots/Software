/**
  ******************************************************************************
  * @file    pid_regulator.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          PID reulator component of the Motor Control SDK.
  * 
  *  The PID regulator component provides the functions needed to implement 
  * a proportional–integral–derivative controller. 
  * 
  * See @link PIDRegulator @endlink for more details.
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
  * @ingroup PIDRegulator
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PIDREGULATOR_H
#define PIDREGULATOR_H

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup PIDRegulator
  * @{
  */

/**
  * @brief Handle of a PID component
  *
  * This structure holds all the parameters needed to implement 
  * a proportional-integral-derivative controller function. 
  * It also stores upper and lower limits used to saturate the 
  * integral term and the output values of the controller. 
  * 
  * A pointer on a structure of this type is passed to each 
  * function of the @ref PIDRegulator "PID Regulator component".
  */
typedef struct
{
  int16_t   hDefKpGain;           /**< @brief Default value of @f$K_{pg}@f$, the numerator of the proportional gain. 
                                    *
                                    *  The value of this field is copied into #hKpGain when the component is initialized.
                                    * @see PID_HandleInit(). 
                                    */
  int16_t   hDefKiGain;           /**< @brief Default value of @f$K_{ig}@f$, the numerator if the integral gain 
                                    *
                                    *  The value of this field is copied into #hKiGain when the component is initialized.
                                    * @see PID_HandleInit(). 
                                    */
  int16_t   hKpGain;              /**< @brief @f$K_{pg}@f$, numerator of the proportional gain of the PID Regulator component */
  int16_t   hKiGain;              /**< @brief @f$K_{ig}@f$, numerator of the integral gain of the PID Regulator component */
  int32_t   wIntegralTerm;        /**< @brief integral term of the PID Regulator
                                    *
                                    * This value is updated on each call to the PI_Controller() or PID_Controller() functions. It 
                                    * contains the integral term before being divided by @f$K_{id}@f$, the divisor of the integral
                                    * gain:
                                    * 
                                    * @f[
                                    * K_{ig}\times\sum_{k=0}^t{\epsilon_k}
                                    * @f]
                                    * 
                                    * This field is reset to 0 when the component is initialized.
                                    * @see PID_HandleInit(). 
                                    */
  int32_t   wUpperIntegralLimit;  /**< @brief Upper limit used to saturate the integral term of the PID Regulator
                                    *
                                    * The integral term, #wIntegralTerm is capped to the value of this field.
                                    */
  int32_t   wLowerIntegralLimit;  /**< @brief Lower limit used to saturate the integral term of the PID Regulator
                                    *
                                    * The integral term, #wIntegralTerm is floored to the value of this field.
                                    */
  int16_t   hUpperOutputLimit;    /**< @brief Upper limit used to saturate the output of the PID Regulator
                                    *
                                    * The output of the PI or PID regulator function is capped to the valud of this field.
                                    * @see PI_Controller() and PID_Controller()
                                    */
  int16_t   hLowerOutputLimit;    /**< @brief Lower limit used to saturate the output of the PID Regulator
                                    *
                                    * The output of the PI or PID regulator function is floored to the valud of this field.
                                    * @see PI_Controller() and PID_Controller()
                                    */
  uint16_t  hKpDivisor;           /**< @brief @f$K_{pd}@f$, divisor of the proportional gain
                                    *
                                    * Used in conjuction with @f$K_{pg}@f$, the proportional gain numerator to allow for obtaining
                                    * fractional gain values.
                                    * 
                                    * If #FULL_MISRA_C_COMPLIANCY is not defined the divisor is implemented through
                                    * algebrical right shifts to speed up the execution of the controller functions. Only in this
                                    * case does this parameter specify the number of right shifts to be executed.
                                    */
  uint16_t  hKiDivisor;           /**< @brief @f$K_{id}@f$, divisor of the integral gain gain 
                                    * 
                                    * Used in conjuction with @f$K_{ig}@f$, the integral gain numerator to allow for obtaining 
                                    * fractional gain values.
                                    * 
                                    * If #FULL_MISRA_C_COMPLIANCY is not defined  the divisor is implemented through
                                    * algebrical right shifts to speed up the execution of the controller functions. Only in this
                                    * case does this parameter specify the number of right shifts to be executed.
                                    */
  uint16_t  hKpDivisorPOW2;       /**< @brief @f$K_{pd}@f$, divisor of the proportional gain, expressed as power of 2.
                                    *
                                    * E.g. if the divisor of the proportional gain is 512, the value of this field is 9 
                                    * as @f$2^9 = 512@f$ 
                                    */
  uint16_t  hKiDivisorPOW2;       /**< @brief @f$K_{id}@f$, divisor of the integral gain, expressed as power of 2.
                                    *
                                    * E.g. if the divisor of the integral gain is 512, the value of this field is 9 
                                    * as @f$2^9 = 512@f$ 
                                    */
  int16_t   hDefKdGain;           /**< @brief Default value of @f$K_{dg}@f$, the numerator of the derivative gain. 
                                    *
                                    *  The value of this field is copied into #hKdGain when the component is initialized.
                                    * @see PID_HandleInit(). 
                                    */
  int16_t   hKdGain;              /**< @brief @f$K_{dg}@f$, numerator of the derivative gain of the PID Regulator component */
  uint16_t  hKdDivisor;           /**< @brief @f$K_{dd}@f$, divisor of the derivative gain gain 
                                    * 
                                    * Used in conjuction with @f$K_{dg}@f$, the derivative gain numerator to allow for obtaining 
                                    * fractional gain values.
                                    * 
                                    * If #FULL_MISRA_C_COMPLIANCY is not defined  the divisor is implemented through
                                    * algebrical right shifts to speed up the execution of the controller functions. Only in this
                                    * case does this parameter specify the number of right shifts to be executed.
                                    */
  uint16_t  hKdDivisorPOW2;       /**< @brief @f$K_{dd}@f$, divisor of the derivative gain, expressed as power of 2.
                                    *
                                    * E.g. if the divisor of the integral gain is 512, the value of this field is 9 
                                    * as @f$2^9 = 512@f$ 
                                    */
  int32_t   wPrevProcessVarError; /**< @brief previous process variable used by the derivative part of the PID component 
                                    *
                                    * This value is updated on each call to the PI_Controller() or PID_Controller() functions. 
                                    * 
                                    * This field is reset to 0 when the component is initialized.
                                    * @see PID_HandleInit().  
                                    */
} PID_Handle_t;

/* Initializes the handle of a PID component */
void PID_HandleInit(PID_Handle_t *pHandle);

/* Sets the numerator of Kp, the proportional gain of a PID component */
void PID_SetKP(PID_Handle_t *pHandle, int16_t hKpGain);

/* Updates the numerator of Ki, the integral gain of a PID component */
void PID_SetKI(PID_Handle_t *pHandle, int16_t hKiGain);

/* Returns the numerator of Kp, the proportional gain of a PID component */
int16_t PID_GetKP(PID_Handle_t *pHandle);

/* Returns the numerator of Ki, the integral gain of a PID component */
int16_t PID_GetKI(PID_Handle_t *pHandle);

/* Returns the default value of the numerator of Kp, the proportional gain of a PID component */
int16_t PID_GetDefaultKP(PID_Handle_t *pHandle);

/* Returns the default value of the numerator of Ki, the integral gain of a PID component */
int16_t PID_GetDefaultKI(PID_Handle_t *pHandle);

/* Sets the value of the integral term of a PID component */
void PID_SetIntegralTerm(PID_Handle_t *pHandle, int32_t wIntegralTermValue);

/* Returns the divisor of Kp, the proportional gain of a PID component */
uint16_t PID_GetKPDivisor(PID_Handle_t *pHandle);

/* Returns the power of two that makes the divisor of Kp, the proportional gain of a PID component */
uint16_t PID_GetKPDivisorPOW2(PID_Handle_t *pHandle);

/* Sets the power of two that makes the divisor of Kp, the proportional gain of a PID component */
void PID_SetKPDivisorPOW2(PID_Handle_t *pHandle, uint16_t hKpDivisorPOW2);

/* Returns the divisor of Ki, the integral gain of a PID component */
uint16_t PID_GetKIDivisor(PID_Handle_t *pHandle);

/* Returns the power of two that makes the divisor of Ki, the inegral gain of a PID component */
uint16_t PID_GetKIDivisorPOW2(PID_Handle_t * pHandle);

/* Sets the power of two that makes the divisor of Ki, the integral gain of a PID component */
void PID_SetKIDivisorPOW2(PID_Handle_t *pHandle, uint16_t hKiDivisorPOW2);

/* Sets the lower limit of the integral term of a PID component */
void PID_SetLowerIntegralTermLimit(PID_Handle_t *pHandle, int32_t wLowerLimit);

/* Sets the upper limit of the integral term of a PID component */
void PID_SetUpperIntegralTermLimit(PID_Handle_t *pHandle, int32_t wUpperLimit);

/* Sets the lower output limit of a PID component */
void PID_SetLowerOutputLimit(PID_Handle_t *pHandle, int16_t hLowerLimit);

/* Sets the upper output limit of a PID component */
void PID_SetUpperOutputLimit(PID_Handle_t *pHandle, int16_t hUpperLimit);

/* Sets the value of the previous process error of a PID component */
void PID_SetPrevError(PID_Handle_t *pHandle, int32_t wPrevProcessVarError);

/* Sets the numerator of Kd, the derivative gain of a PID component */
void PID_SetKD(PID_Handle_t *pHandle, int16_t hKdGain);

/* Returns the numerator of Kd, the derivativz gain of a PID component */
int16_t PID_GetKD(PID_Handle_t *pHandle);

/* Returns the divisor of Kd, the derivative gain of a PID component */
uint16_t PID_GetKDDivisor(PID_Handle_t *pHandle);

/* Returns the power of two that makes the divisor of Kd, the derivative gain of a PID component */
uint16_t PID_GetKDDivisorPOW2(PID_Handle_t *pHandle);

/* Sets the power of two that makes the divisor of Kd, the derivative gain of a PID component */
void PID_SetKDDivisorPOW2(PID_Handle_t *pHandle, uint16_t hKdDivisorPOW2);

/* 
 * Computes the output of a PI Regulator component, sum of its proportional 
 * and integral terms
 */
int16_t PI_Controller(PID_Handle_t *pHandle, int32_t wProcessVarError);

/* 
 * Computes the output of a PID Regulator component, sum of its proportional, 
 * integral and derivative terms
 */
int16_t PID_Controller(PID_Handle_t *pHandle, int32_t wProcessVarError);

/**
  * @}
  */

/**
  * @}
  */

#endif /*PIDREGULATOR_H*/

/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
