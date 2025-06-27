/**
  ******************************************************************************
  * @file    pid_regulator.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the PID regulator component of the Motor Control SDK.
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

/* Includes ------------------------------------------------------------------*/
#include "pid_regulator.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/**
  * @defgroup PIDRegulator PID Regulator
  * @brief PID Regulator component of the Motor Control SDK
  * 
  * The PID regulator component implements the following two control functions:
  *
  * * A simple proportional-integral controller, implemented by the PI_Controller() function:
  * @f[
  * r(t_k) = K_p \times \epsilon(t_k) + K_i \times \sum_{j=0}^k\epsilon(t_j)
  * @f]
  * * A complete proportional–integral–derivative controller, implemented by the PID_Controller() function:
  * @f[
  * r(t_k) = K_p \times \epsilon(t_k) + K_i \times \sum_{j=0}^k\epsilon(t_j) + K_d \times (\epsilon(t_k) - \epsilon(t_{k-1}))
  * @f]
  *
  * The proportional, integral and derivative gains are expressed as rational numbers, with a gain (numerator)
  * and a divisor (denominator) parameters:
  *
  * * Proportional gain: @f$ K_{p} = K_{pg} / K_{pd} @f$
  * * Integral gain: @f$ K_{i} = K_{ig} / K_{id} @f$
  * * Derivative gain: @f$ K_{d} = K_{dg} / K_{dd} @f$
  *
  * Each of the gain numerator and divisor parameters, @f$K_{{p}g}@f$, @f$K_{{i}g}@f$, @f$K_{{d}g}@f$, @f$K_{{p}d}@f$,
  * @f$K_{id}@f$, @f$K_{dd}@f$, can be set, at run time and independently, via the PID_SetKP(), PID_SetKI(), PID_SetKD(), 
  * PID_SetKPDivisorPOW2(), PID_SetKIDivisorPOW2() and PID_SetKDDivisorPOW2() functions, respectively. 
  * 
  * A PID Regulator component needs to be initialized before it can be used. This is done with the PID_HandleInit() 
  * function that sets the intergral term and the derivative term base (the previous value of the process error input) 
  * to 0 and that also resets the numerators of the proportional, integral and derivative gains to their default values. 
  * These default values are literally written in the code of the application, so they are set at compile time. They 
  * can be retrieved with the PID_GetDefaultKP(), PID_GetDefaultKI() and PID_GetDefaultKD() functions. 
  * 
  * The controller functions implemented by the PI_Controller() and the PID_Controller() functions are based on 16-bit
  * integer arithmetics: the gains are expressed as fractional numbers, with 16-bit numerators and denominators. And the 
  * controller output values returned by these functions are also 16-bit integers. This makes it possible to use this 
  * component efficiently on all STM2 MCUs. 
  * 
  * To keep the computed values within limits, the component features the possibility to constrain the integral term 
  * within a range of values bounded by the PID_SetLowerIntegralTermLimit() and PID_SetUpperIntegralTermLimit() functions.
  * 
  * The output valud of the controller can also be bounded between a high and a low limit thanks to the 
  * PID_SetLowerOutputLimit() and PID_SetUpperOutputLimit() functions.
  *
  * Hanlding a process with a PID Controller may require some adjustment to cope with specific situations. To that end, the 
  * PID regulator component provides functions to set the integral term (PID_SetIntegralTerm()) or to set the value of the 
  * previous process error (PID_SetPrevError()).
  * 
  * See the [PID chapter of the User Manual](PID_regulator_theoretical_background.md) for more details on the theoretical
  * background of this regulator.
  * @{
  */

/**
  * @brief  Initializes the handle of a PID component
  * @param  pHandle A Handle on the PID component to initialize
  * 
  * The integral term and the derivative base of the PID component are 
  * set to zero. 
  * 
  * The numerators of the proportional, integral and derivative gains 
  * are set to their default values. These default values are the ones 
  * set to the PID_Handle_t::hDefKpGain, PID_Handle_t::hDefKiGain and
  * PID_Handle_t::hDefKdGain fields of the PID_Handle_t structure.
  */
__weak void PID_HandleInit(PID_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_PID_REG
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->hKpGain =  pHandle->hDefKpGain;
    pHandle->hKiGain =  pHandle->hDefKiGain;
    pHandle->hKdGain =  pHandle->hDefKdGain;
    pHandle->wIntegralTerm = 0;
    pHandle->wPrevProcessVarError = 0;
#ifdef NULL_PTR_CHECK_PID_REG
  }
#endif
}

/**
  * @brief  Sets @f$K_{pg}@f$, the numerator of the proportional gain of a PID component
  * @param  pHandle Handle on the PID component
  * @param  hKpGain New @f$K_{pg}@f$ value
  */
__weak void PID_SetKP(PID_Handle_t *pHandle, int16_t hKpGain)
{
#ifdef NULL_PTR_CHECK_PID_REG
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->hKpGain = hKpGain;
#ifdef NULL_PTR_CHECK_PID_REG
  }
#endif
}

/**
  * @brief  Sets the @f$K_{ig}@f$, the numrerator of the integral gain of a PID component
  * @param  pHandle Handle on the PID component
  * @param  hKiGain new @f$K_{ig}@f$ value
  */
__weak void PID_SetKI(PID_Handle_t *pHandle, int16_t hKiGain)
{
#ifdef NULL_PTR_CHECK_PID_REG
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->hKiGain = hKiGain;
#ifdef NULL_PTR_CHECK_PID_REG
  }
#endif
}

/**
  * @brief  Returns @f$K_{pg}@f$, the numerator of the proportional gain of a PID component
  * @param  pHandle Handle on the PID component
  */
__weak int16_t PID_GetKP(PID_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_PID_REG
  return ((MC_NULL == pHandle) ? 0 : pHandle->hKpGain);
#else
  return (pHandle->hKpGain);
#endif
}

/**
  * @brief  Returns @f$K_{ig}@f$, the numrerator of the integral gain of a PID component
  * @param  pHandle Handle on the PID component
  */
__weak int16_t PID_GetKI(PID_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_PID_REG
  return ((MC_NULL == pHandle) ? 0 : pHandle->hKiGain);
#else
  return (pHandle->hKiGain);
#endif
}

/**
  * @brief  Returns the default @f$K_{pg}@f$ value, the numerator of the proportional
  *         gain of a PID component
  * @param  pHandle Handle on the PID component
  * 
  *  The default @f$K_{pg}@f$ value is the one that is being used at startup time,
  * when the MCU is reset or when the PID_HandleInit() function gets called. When 
  * any of the last two event occurs, any proportional gain numerator that may 
  * have been previously set with the PID_SetKP() function is replaced by this 
  * default value.
  */
__weak int16_t PID_GetDefaultKP(PID_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_PID_REG
  return ((MC_NULL == pHandle) ? 0 : pHandle->hDefKpGain);
#else
  return (pHandle->hDefKpGain);
#endif
}

/**
  * @brief  Returns the default @f$K_{ig}@f$ value, the numerator of the integral
  *         gain of a PID component
  * @param  pHandle Handle on the PID component
  * 
  *  The default @f$K_{ig}@f$ value is the one that is being used at startup time,
  * when the MCU is reset or when the PID_HandleInit() function gets called. When 
  * any of the last two event occurs, any gain that may have been previously set 
  * with the PID_SetKI() function is replaced by this default value.
  */
__weak int16_t PID_GetDefaultKI(PID_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_PID_REG
  return ((MC_NULL == pHandle) ? 0 : pHandle->hDefKiGain);
#else
  return (pHandle->hDefKiGain);
#endif
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
  * @brief  Sets the value of the integral term of a PID component
  * @param  pHandle Handle on the PID component
  * @param  wIntegralTermValue new integral term value multiplied by the divisor 
  *                            of the integral gain
  * 
  * If @f$T_{i}@f$ is the target integral term, the @p wIntegralTermValue term is stored
  * before the division by @f$K_{id}@f$ to maximize the available dynamics.
  * 
  * @attention @p wIntegralTermValue divided by @f$K_{id}@f$ must fit in a 16-bit signed 
  * integer value.
  */
__weak void PID_SetIntegralTerm(PID_Handle_t *pHandle, int32_t wIntegralTermValue)
{
#ifdef NULL_PTR_CHECK_PID_REG
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->wIntegralTerm = wIntegralTermValue;
#ifdef NULL_PTR_CHECK_PID_REG
  }
#endif
  return;
}

/**
  * @brief  Returns @f$K_{pd}@f$, the divisor of the proportional gain of a PID component
  * @param  pHandle Handle on the PID component
  *
  * The divisors that make the gains of the @ref PIDRegulator "PID regulator component" are 
  * powers of two. 
  * 
  * @sa PID_GetKPDivisorPOW2()
  */
__weak uint16_t PID_GetKPDivisor(PID_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_PID_REG
  return ((MC_NULL == pHandle) ? 0U : pHandle->hKpDivisor);
#else
  return (pHandle->hKpDivisor);
#endif
}

/**
  * @brief  Returns the power of two that makes @f$K_{pd}@f$, the divisor of the proportional 
  *         gain of a PID component
  * @param  pHandle Handle on the PID component
  * 
  * The divisors that make the gains of the @ref PIDRegulator "PID regulator component" are 
  * powers of two. 
  * 
  * @sa PID_GetKPDivisor()
  */
__weak uint16_t PID_GetKPDivisorPOW2(PID_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_PID_REG
  return ((MC_NULL == pHandle) ? 0U : pHandle->hKpDivisorPOW2);
#else
  return (pHandle->hKpDivisorPOW2);
#endif
}

/**
  * @brief  Sets the power of two that makes @f$K_{pd}@f$, the divisor of the proportional gain
  *         of a PID component
  * @param  pHandle Handle on the PID component
  * @param  hKpDivisorPOW2 new @f$K_{pd}@f$ divisor value, expressed as power of 2
  * 
  * The divisors that make the gains of the @ref PIDRegulator "PID regulator component" are 
  * powers of two. 
  * 
  * This function sets @f$K_{pd}@f$ to 2 to the power of @p hKpDivisorPOW2.
  */
__weak void PID_SetKPDivisorPOW2(PID_Handle_t *pHandle, uint16_t hKpDivisorPOW2)
{
#ifdef NULL_PTR_CHECK_PID_REG
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->hKpDivisorPOW2 = hKpDivisorPOW2;
    pHandle->hKpDivisor = (((uint16_t)1) << hKpDivisorPOW2);
#ifdef NULL_PTR_CHECK_PID_REG
  }
#endif
}

/**
  * @brief  Returns @f$K_{id}@f$, the divisor of the integral gain of a PID component
  * @param  pHandle Handle on the PID component
  *
  * The divisors that make the gains of the @ref PIDRegulator "PID regulator component" are 
  * powers of two. 
  * 
  * @sa PID_GetKIDivisorPOW2()
  */
__weak uint16_t PID_GetKIDivisor(PID_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_PID_REG
  return ((MC_NULL == pHandle) ? 0U : pHandle->hKiDivisor);
#else
  return (pHandle->hKiDivisor);
#endif
}

/**
  * @brief Returns the power of two that makes @f$K_{id}@f$, the divisor of the integral 
  *        gain of a PID component
  * @param  pHandle Handle on the PID component
  * 
  * The divisors that make the gains of the @ref PIDRegulator "PID regulator component" are 
  * powers of two. 
  * 
  * @sa PID_GetKIDivisor()
  */
__weak uint16_t PID_GetKIDivisorPOW2(PID_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_PID_REG
  return ((MC_NULL == pHandle) ? 0U : pHandle->hKiDivisorPOW2);
#else
  return (pHandle->hKiDivisorPOW2);
#endif
}

/**
  * @brief  Sets the power of two that makes @f$K_{id}@f$, the divisor of the integral gain
  *         of a PID component
  * @param  pHandle Handle on the PID component
  * @param  hKiDivisorPOW2 new @f$K_{id}@f$ divisor value, expressed as power of 2
  * 
  * The divisors that make the gains of the @ref PIDRegulator "PID regulator component" are 
  * powers of two. 
  * 
  * This function sets @f$K_{id}@f$ to 2 to the power of @p hKiDivisorPOW2. 
  * 
  * Note that the upper and lower limits of the integral term are also updated to
  * accept any 16-bit value. If the limits of the integral term need to be different 
  * use the PID_SetUpperIntegralTermLimit() and PID_SetLowerIntegralTermLimit() functions
  * after this one.
  */
__weak void PID_SetKIDivisorPOW2(PID_Handle_t *pHandle, uint16_t hKiDivisorPOW2)
{
#ifdef NULL_PTR_CHECK_PID_REG
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    uint32_t wKiDiv = (((uint32_t)1) << hKiDivisorPOW2);
    pHandle->hKiDivisorPOW2 = hKiDivisorPOW2;
    pHandle->hKiDivisor = (uint16_t)wKiDiv;
    PID_SetUpperIntegralTermLimit(pHandle, (int32_t)INT16_MAX * (int32_t)wKiDiv);
    PID_SetLowerIntegralTermLimit(pHandle, (int32_t)(-INT16_MAX) * (int32_t)wKiDiv);
#ifdef NULL_PTR_CHECK_PID_REG
  }
#endif
}

/**
  * @brief  Sets the lower limit of the integral term of a PID component
  * 
  * @param  pHandle Handle on the PID component
  * @param  wLowerLimit new lower integral term limit multiplied by the divisor 
  *                     of the integral gain
  * 
  * If @f$T_{iL}@f$ is the target lower limit, the @p wLowerLimit parameter must 
  * be set to @f$T_{iL}\times K_{id}@f$. This is because the limit is checked before
  * applying the divisor in the PI_Controller() and PID_Controller() controller 
  * functions.
  * 
  * When the PI or PID controller is executed, the value of the integral term is floored
  * to this value.
  * 
  * @attention @p wLowerLimit divided by @f$K_{id}@f$ must fit in a 16-bit signed 
  * integer value.
  */
__weak void PID_SetLowerIntegralTermLimit(PID_Handle_t *pHandle, int32_t wLowerLimit)
{
#ifdef NULL_PTR_CHECK_PID_REG
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->wLowerIntegralLimit = wLowerLimit;
#ifdef NULL_PTR_CHECK_PID_REG
  }
#endif
}

/**
  * @brief  Sets the upper limit of the integral term of a PID component
  * 
  * @param  pHandle Handle on the PID component
  * @param  wUpperLimit new upper integral term limit multiplied by the divisor 
  *                     of the integral gain
  * 
  * If @f$T_{iU}@f$ is the target upper limit, the @p wUpperLimit parameter must 
  * be set to @f$T_{iU}\times K_{id}@f$. This is because the limit is checked before
  * applying the divisor in the PI_Controller() and PID_Controller() controller 
  * functions.
  * 
  * When the controller is executed, the value of the integral term is capped to 
  * this value.
  * 
  * @attention @p wUpperLimit divided by @f$K_{id}@f$ must fit in a 16-bit signed 
  * integer value.
  */
__weak void PID_SetUpperIntegralTermLimit(PID_Handle_t *pHandle, int32_t wUpperLimit)
{
#ifdef NULL_PTR_CHECK_PID_REG
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->wUpperIntegralLimit = wUpperLimit;
#ifdef NULL_PTR_CHECK_PID_REG
  }
#endif
}

/**
  * @brief  Sets the lower output limit of a PID component
  * 
  * @param  pHandle Handle on the PID component
  * @param  hLowerLimit new lower limit of the output value
  * 
  * When the controller is executed, the value of its output is floored to this value.
  */
__weak void PID_SetLowerOutputLimit(PID_Handle_t *pHandle, int16_t hLowerLimit)
{
#ifdef NULL_PTR_CHECK_PID_REG
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->hLowerOutputLimit = hLowerLimit;
#ifdef NULL_PTR_CHECK_PID_REG
  }
#endif
}

/**
  * @brief  Sets the upper output limit of a PID component
  * 
  * @param  pHandle Handle on the PID component
  * @param  hUpperLimit: new upper limit of the output value
  * 
  * When the controller is executed, the value of its output is capped to this value.
  */
__weak void PID_SetUpperOutputLimit(PID_Handle_t *pHandle, int16_t hUpperLimit)
{
#ifdef NULL_PTR_CHECK_PID_REG
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->hUpperOutputLimit = hUpperLimit;
#ifdef NULL_PTR_CHECK_PID_REG
  }
#endif
}

/**
  * @brief  Sets the value of the previous process error of a PID component
  * 
  * @param  pHandle Handle on the PID component
  * @param  wPrevProcessVarError new value of the previous error variable
  */
__weak void PID_SetPrevError(PID_Handle_t *pHandle, int32_t wPrevProcessVarError)
{
#ifdef NULL_PTR_CHECK_PID_REG
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->wPrevProcessVarError = wPrevProcessVarError;
#ifdef NULL_PTR_CHECK_PID_REG
  }
#endif
  return;
}

/**
  * @brief  Sets @f$K_{dg}@f$, the numerator of the derivative gain of a PID component
  * @param  pHandle Handle on the PID component
  * @param  hKpGain New @f$K_{dg}@f$ value
  */
__weak void PID_SetKD(PID_Handle_t *pHandle, int16_t hKdGain)
{
#ifdef NULL_PTR_CHECK_PID_REG
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->hKdGain = hKdGain;
#ifdef NULL_PTR_CHECK_PID_REG
  }
#endif
}

/**
  * @brief  Returns @f$K_{dg}@f$, the numerator of the derivative gain of a PID component
  * @param  pHandle Handle on the PID component
  */
__weak int16_t PID_GetKD(PID_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_PID_REG
  return ((MC_NULL == pHandle) ? 0 : pHandle->hKdGain);
#else
  return (pHandle->hKdGain);
#endif
}

/**
  * @brief  Returns @f$K_{dd}@f$, the divisor of the derivative gain of a PID component
  * @param  pHandle Handle on the PID component
  *
  * The divisors that make the gains of the @ref PIDRegulator "PID regulator component" are 
  * powers of two. 
  * 
  * @sa PID_GetKDDivisorPOW2()
  */
__weak uint16_t PID_GetKDDivisor(PID_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_PID_REG
  return ((MC_NULL == pHandle) ? 0U : pHandle->hKdDivisor);
#else
  return (pHandle->hKdDivisor);
#endif
}

/**
  * @brief  Returns the power of two that makes @f$K_{dd}@f$, the divisor of the derivative 
  *         gain of a PID component
  * @param  pHandle Handle on the PID component
  * 
  * The divisors that make the gains of the @ref PIDRegulator "PID regulator component" are 
  * powers of two. 
  * 
  * @sa PID_GetKDDivisor()
  */
__weak uint16_t PID_GetKDDivisorPOW2(PID_Handle_t * pHandle)
{
#ifdef NULL_PTR_CHECK_PID_REG
  return ((MC_NULL == pHandle) ? 0U : pHandle->hKdDivisorPOW2);
#else
  return (pHandle->hKdDivisorPOW2);
#endif
}

/**
  * @brief  Sets the power of two that makes @f$K_{dd}@f$, the divisor of the derivative gain
  *         of a PID component
  * @param  pHandle Handle on the PID component
  * @param  hKdDivisorPOW2 new @f$K_{dd}@f$ divisor value, expressed as power of 2
  * 
  * The divisors that make the gains of the @ref PIDRegulator "PID regulator component" are 
  * powers of two. 
  * 
  * This function sets @f$K_{dd}@f$ to 2 to the power of @p hKdDivisorPOW2.
  */
__weak void PID_SetKDDivisorPOW2(PID_Handle_t *pHandle, uint16_t hKdDivisorPOW2)
{
#ifdef NULL_PTR_CHECK_PID_REG
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->hKdDivisorPOW2 = hKdDivisorPOW2;
    pHandle->hKdDivisor = (((uint16_t)1) << hKdDivisorPOW2);
#ifdef NULL_PTR_CHECK_PID_REG
  }
#endif
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
  * @brief  Computes the output of a PI Regulator component, sum of its proportional and 
  *         integral terms
  * 
  * @param  pHandle Handle on the PID component
  * @param  wProcessVarError current process variable error (the reference value minus the 
  *                          present process variable value)
  * @retval computed PI controller output
  * 
  * This function implements the proportional-integral controller function described by the 
  * @ref PIDRegulator "PID regulator component". The integral term is saturated by the upper 
  * and lower intergral term limit values before it is added to the proportional term. 
  * 
  * The resulting value is then saturated by the upper and lower output limit values before 
  * being returned.
  */
__weak int16_t PI_Controller(PID_Handle_t *pHandle, int32_t wProcessVarError)
{
  int16_t returnValue;
#ifdef NULL_PTR_CHECK_PID_REG
  if (MC_NULL == pHandle)
  {
    returnValue = 0;
  }
  else
  {
#endif
    int32_t wProportional_Term;
    int32_t wIntegral_Term;
    int32_t wOutput_32;
    int32_t wIntegral_sum_temp;
    int32_t wDischarge = 0;
    int16_t hUpperOutputLimit = pHandle->hUpperOutputLimit;
    int16_t hLowerOutputLimit = pHandle->hLowerOutputLimit;

    /* Proportional term computation*/
    wProportional_Term = pHandle->hKpGain * wProcessVarError;

    /* Integral term computation */
    if (0 == pHandle->hKiGain)
    {
      pHandle->wIntegralTerm = 0;
    }
    else
    {
      wIntegral_Term = pHandle->hKiGain * wProcessVarError;
      wIntegral_sum_temp = pHandle->wIntegralTerm + wIntegral_Term;

      if (wIntegral_sum_temp < 0)
      {
        if (pHandle->wIntegralTerm > 0)
        {
          if (wIntegral_Term > 0)
          {
            wIntegral_sum_temp = INT32_MAX;
          }
          else
          {
            /* Nothing to do */
          }
        }
        else
        {
          /* Nothing to do */
        }
      }
      else
      {
        if (pHandle->wIntegralTerm < 0)
        {
          if (wIntegral_Term < 0)
          {
            wIntegral_sum_temp = -INT32_MAX;
          }
          else
          {
            /* Nothing to do */
          }
        }
        else
        {
          /* Nothing to do */
        }
      }

      if (wIntegral_sum_temp > pHandle->wUpperIntegralLimit)
      {
        pHandle->wIntegralTerm = pHandle->wUpperIntegralLimit;
      }
      else if (wIntegral_sum_temp < pHandle->wLowerIntegralLimit)
      {
        pHandle->wIntegralTerm = pHandle->wLowerIntegralLimit;
      }
      else
      {
        pHandle->wIntegralTerm = wIntegral_sum_temp;
      }
    }

#ifndef FULL_MISRA_C_COMPLIANCY_PID_REGULATOR
    /* WARNING: the below instruction is not MISRA compliant, user should verify
               that Cortex-M3 assembly instruction ASR (arithmetic shift right)
               is used by the compiler to perform the shifts (instead of LSR
               logical shift right)*/
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
    wOutput_32 = (wProportional_Term >> pHandle->hKpDivisorPOW2) + (pHandle->wIntegralTerm >> pHandle->hKiDivisorPOW2);
#else
    wOutput_32 = (wProportional_Term / (int32_t)pHandle->hKpDivisor)
              + (pHandle->wIntegralTerm / (int32_t)pHandle->hKiDivisor);
#endif

    if (wOutput_32 > hUpperOutputLimit)
    {
      wDischarge = hUpperOutputLimit - wOutput_32;
      wOutput_32 = hUpperOutputLimit;
    }
    else if (wOutput_32 < hLowerOutputLimit)
    {
      wDischarge = hLowerOutputLimit - wOutput_32;
      wOutput_32 = hLowerOutputLimit;
    }
    else
    {
      /* Nothing to do here */
    }

    pHandle->wIntegralTerm += wDischarge;
    returnValue = (int16_t)wOutput_32;
#ifdef NULL_PTR_CHECK_PID_REG
  }
#endif
  return (returnValue);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
  * @brief  Computes the output of a PID Regulator component, sum of its proportional,
  *         integral and derivative terms
  * 
  * @param  pHandle Handle on the PID component
  * @param  wProcessVarError current process variable error (the reference value minus the 
  *                          present process variable value)
  * @retval computed PID controller output
  * 
  * This function implements the proportional-integral-derivative controller function described  
  * by the @ref PIDRegulator "PID regulator component". The integral term is saturated by  
  * the upper and lower intergral term limit values before it is added to the proportional term
  * and derivative terms. 
  * 
  * The resulting value is then saturated by the upper and lower output limit values before 
  * being returned.
  */
__weak int16_t PID_Controller(PID_Handle_t *pHandle, int32_t wProcessVarError)
{
  int16_t returnValue;
#ifdef NULL_PTR_CHECK_PID_REG
  if (MC_NULL == pHandle)
  {
    returnValue = 0;
  }
  else
  {
#endif
    int32_t wDifferential_Term;
    int32_t wDeltaError;
    int32_t wTemp_output;

    if (0 == pHandle->hKdGain) /* derivative terms not used */
    {
      wTemp_output = PI_Controller(pHandle, wProcessVarError);
    }
    else
    {
      wDeltaError = wProcessVarError - pHandle->wPrevProcessVarError;
      wDifferential_Term = pHandle->hKdGain * wDeltaError;

#ifndef FULL_MISRA_C_COMPLIANCY_PID_REGULATOR
      /* WARNING: the below instruction is not MISRA compliant, user should verify
         that Cortex-M3 assembly instruction ASR (arithmetic shift right)
         is used by the compiler to perform the shifts (instead of LSR
         logical shift right)*/
      //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
      wDifferential_Term >>= pHandle->hKdDivisorPOW2;
#else
      wDifferential_Term /= ((int32_t)pHandle->hKdDivisor);
#endif

      pHandle->wPrevProcessVarError = wProcessVarError;

      wTemp_output = PI_Controller(pHandle, wProcessVarError) + wDifferential_Term;

      if (wTemp_output > pHandle->hUpperOutputLimit)
      {
        wTemp_output = pHandle->hUpperOutputLimit;
      }
      else if (wTemp_output < pHandle->hLowerOutputLimit)
      {
        wTemp_output = pHandle->hLowerOutputLimit;
      }
      else
      {
        /* Nothing to do */
      }
    }
    returnValue = (int16_t) wTemp_output;
#ifdef NULL_PTR_CHECK_PID_REG
  }
#endif
  return (returnValue);
}
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
