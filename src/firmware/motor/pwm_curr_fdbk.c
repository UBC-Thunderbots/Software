/**
  ******************************************************************************
  * @file    pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the following features
  *          of the PWM & Current Feedback component of the Motor Control SDK:
  *
  *           * current sensing
  *           * regular ADC conversion execution
  *           * space vector modulation
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
  * @ingroup pwm_curr_fdbk
  */

/* Includes ------------------------------------------------------------------*/
#include "firmware/motor/mcsdk/pwm_curr_fdbk.h"

#include "firmware/motor/mc_math.h"
#include "firmware/motor/mcsdk/mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup pwm_curr_fdbk PWM & Current Feedback
  *
  * @brief PWM & Current Feedback components of the Motor Control SDK.
  *
  * These components fulfill two functions in a Motor Control subsystem:
  *
  * - The generation of the Space Vector Pulse Width Modulation on the motor's phases
  * - The sampling of the actual motor's phases current
  *
  * Both these features are closely related as the instants when the values of the phase currents
  * should be sampled by the ADC channels are basically triggered by the timers used to generate
  * the duty cycles for the PWM.
  *
  * Several implementation of PWM and Current Feedback components are provided by the Motor Control
  * SDK to account for the specificities of the application:
  *
  * - The selected MCU: the number of ADCs available on a given MCU, the presence of internal
  * comparators or OpAmps, for instance, lead to different implementation of this feature
  * - The Current sensing topology also has an impact on the firmware: implementations are provided
  * for Insulated Current Sensors, Single Shunt and Three Shunt resistors current sensing topologies
  *
  * The choice of the implementation mostly depend on these two factors and is performed by the
  * Motor Control Workbench tool.
  *
  * All these implementations are built on a base PWM & Current Feedback component that they extend
  * and that provides the functions and data that are common to all of them. This base component is
  * never used directly as it does not provide a complete implementation of the features. Rather,
  * its handle structure (PWMC_Handle) is reused by all the PWM & Current Feedback specific
  * implementations and the functions it provides form the API of the PWM and Current feedback feature.
  * Calling them results in calling functions of the component that actually implement the feature.
  * See PWMC_Handle for more details on this mechanism.
  * @{
  */

/**
  * @brief  Used to clear variables in CPWMC.
  *
  * @param pHandle: Handler of the current instance of the PWM component.
  */
void PWMC_Clear(PWMC_Handle_t *pHandle)
{
#ifdef NULL_PTR_PWR_CUR_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->IaEst = 0;
    pHandle->IbEst = 0;
    pHandle->IcEst = 0;
    pHandle->LPFIdBuf = 0;
    pHandle->LPFIqBuf = 0;
#ifdef NULL_PTR_PWR_CUR_FDB
  }
#endif
}

/**
  * @brief  Sets the calibrated @p offsets for each of the phases in the @p pHandle handler. In case of single shunt only phase A is relevant.
  *
  */
//cstat !MISRAC2012-Rule-8.13 !RED-func-no-effect
__weak void PWMC_SetOffsetCalib(PWMC_Handle_t *pHandle, PolarizationOffsets_t *offsets)
{
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    pHandle->pFctSetOffsetCalib(pHandle, offsets);
  }
}

/**
  * @brief  Gets the calibrated @p offsets for each of the phases in the @p pHandle handler. In case of single shunt only phase A is relevant.
  *
  */
//cstat !MISRAC2012-Rule-8.13 !RED-func-no-effect
__weak void PWMC_GetOffsetCalib(PWMC_Handle_t *pHandle, PolarizationOffsets_t *offsets)
{
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    pHandle->pFctGetOffsetCalib(pHandle, offsets);
  }
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief Returns the phase current of the motor as read by the ADC (in s16A unit).
  *
  * Returns the current values of phases A & B. Phase C current
  * can be deduced thanks to the formula:
  *
  * @f[
  * I_{C} = -I_{A} - I_{B}
  * @f]
  *
  * @param  pHandle: Handler of the current instance of the PWM component.
  * @param  Iab: Pointer to the structure that will receive motor current
  *         of phases A & B in ElectricalValue format.
  */
//cstat !MISRAC2012-Rule-8.13 !RED-func-no-effect
__weak void PWMC_GetPhaseCurrents(PWMC_Handle_t *pHandle, ab_t *Iab)
{
#ifdef NULL_PTR_PWR_CUR_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->pFctGetPhaseCurrents(pHandle, Iab);
#ifdef NULL_PTR_PWR_CUR_FDB
  }
#endif
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif

/**
  * @brief  Converts input voltages @f$ V_{\alpha} @f$ and @f$ V_{\beta} @f$ into PWM duty cycles
  *         and feed them to the inverter.
  *
  * This function computes the time during which the transistors of each phase are to be switched on in
  * a PWM cycle in order to achieve the reference phase voltage set by @p Valfa_beta. The function then
  * programs the resulting duty cycles in the related timer channels. It also sets the phase current
  * sampling point for the next PWM cycle accordingly.
  *
  * This function is used in the FOC frequency loop and needs to complete itself before the next PWM cycle starts
  * in order for the duty cycles it computes to be taken into account. Failing to do so (for instance because
  * the PWM Frequency is too high) results in the function returning #MC_DURATION which entails a
  * Motor Control Fault that stops the motor.
  *
  * @param  pHandle: Handler of the current instance of the PWM component.
  * @param  Valfa_beta: Voltage Components expressed in the @f$(\alpha, \beta)@f$ reference frame.
  * @retval #MC_NO_ERROR if no error occurred or #MC_DURATION if the duty cycles were
  *         set too late for being taken into account in the next PWM cycle.
  */
__weak uint16_t PWMC_SetPhaseVoltage(PWMC_Handle_t *pHandle, alphabeta_t Valfa_beta)
{
  uint16_t returnValue;
#ifdef NULL_PTR_PWR_CUR_FDB
  if (MC_NULL == pHandle)
  {
    returnValue = 0U;
  }
  else
  {
#endif
    int32_t wX;
    int32_t wY;
    int32_t wZ;
    int32_t wUAlpha;
    int32_t wUBeta;
    int32_t wTimePhA;
    int32_t wTimePhB;
    int32_t wTimePhC;

    wUAlpha = Valfa_beta.alpha * (int32_t)pHandle->hT_Sqrt3;
    wUBeta = -(Valfa_beta.beta * ((int32_t)pHandle->PWMperiod)) * 2;

    wX = wUBeta;
    wY = (wUBeta + wUAlpha) / 2;
    wZ = (wUBeta - wUAlpha) / 2;

    /* Sector calculation from wX, wY, wZ */
    if (wY < 0)
    {
      if (wZ < 0)
      {
        pHandle->Sector = SECTOR_5;
        wTimePhA = (((int32_t)pHandle->PWMperiod) / 4) + ((wY - wZ) / (int32_t)262144);
        wTimePhB = wTimePhA + (wZ / 131072);
        wTimePhC = wTimePhA - (wY / 131072) ;

        pHandle->lowDuty = 1U;
        pHandle->midDuty = 0U;
        pHandle->highDuty = 2U;
      }
      else /* wZ >= 0 */
        if (wX <= 0)
        {
          pHandle->Sector = SECTOR_4;
          wTimePhA = (((int32_t)pHandle->PWMperiod) / 4) + ((wX - wZ) / (int32_t)262144);
          wTimePhB = wTimePhA + (wZ / 131072);
          wTimePhC = wTimePhB - (wX / 131072);

          pHandle->lowDuty = 0U;
          pHandle->midDuty = 1U;
          pHandle->highDuty = 2U;
        }
        else /* wX > 0 */
        {
          pHandle->Sector = SECTOR_3;
          wTimePhA = (((int32_t )pHandle->PWMperiod) / 4)+ ((wY - wX) / (int32_t)262144);
          wTimePhC = wTimePhA - (wY / 131072);
          wTimePhB = wTimePhC + (wX / 131072);

          pHandle->lowDuty = 0U;
          pHandle->midDuty = 2U;
          pHandle->highDuty = 1U;
        }
    }
    else /* wY > 0 */
    {
      if (wZ >= 0)
      {
        pHandle->Sector = SECTOR_2;
        wTimePhA = (((int32_t)pHandle->PWMperiod) / 4) + ((wY - wZ) / (int32_t)262144);
        wTimePhB = wTimePhA + (wZ / 131072);
        wTimePhC = wTimePhA - (wY / 131072);

        pHandle->lowDuty = 2U;
        pHandle->midDuty = 0U;
        pHandle->highDuty = 1U;
      }
      else /* wZ < 0 */
        if ( wX <= 0 )
        {
          pHandle->Sector = SECTOR_6;
          wTimePhA = (((int32_t )pHandle->PWMperiod) / 4) + ((wY - wX) / (int32_t)262144);
          wTimePhC = wTimePhA - (wY / 131072);
          wTimePhB = wTimePhC + (wX / 131072);

          pHandle->lowDuty = 1U;
          pHandle->midDuty = 2U;
          pHandle->highDuty = 0U;
        }
        else /* wX > 0 */
        {
          pHandle->Sector = SECTOR_1;
          wTimePhA = (((int32_t)pHandle->PWMperiod) / 4)+ ((wX - wZ) / (int32_t)262144);
          wTimePhB = wTimePhA + (wZ / 131072);
          wTimePhC = wTimePhB - (wX / 131072);

          pHandle->lowDuty = 2U;
          pHandle->midDuty = 1U;
          pHandle->highDuty = 0U;
        }
    }

    pHandle->CntPhA = (uint16_t)(MAX(wTimePhA, 0));
    pHandle->CntPhB = (uint16_t)(MAX(wTimePhB, 0));
    pHandle->CntPhC = (uint16_t)(MAX(wTimePhC, 0));

    if (1U == pHandle->DTTest)
    {
      /* Dead time compensation */
      if (pHandle->Ia > 0)
      {
        pHandle->CntPhA += pHandle->DTCompCnt;
      }
      else
      {
        pHandle->CntPhA -= pHandle->DTCompCnt;
      }

      if (pHandle->Ib > 0)
      {
        pHandle->CntPhB += pHandle->DTCompCnt;
      }
      else
      {
        pHandle->CntPhB -= pHandle->DTCompCnt;
      }

      if (pHandle->Ic > 0)
      {
        pHandle->CntPhC += pHandle->DTCompCnt;
      }
      else
      {
        pHandle->CntPhC -= pHandle->DTCompCnt;
      }
    }
    returnValue = pHandle->pFctSetADCSampPointSectX(pHandle);
#ifdef NULL_PTR_PWR_CUR_FDB
  }
#endif
  return (returnValue);
}

/**
  * @brief  Switches PWM generation off, inactivating the outputs.
  *
  * @param  pHandle: Handler of the current instance of the PWM component.
  */
//cstat !MISRAC2012-Rule-8.13 !RED-func-no-effect
__weak void PWMC_SwitchOffPWM(PWMC_Handle_t *pHandle)
{
#ifdef NULL_PTR_PWR_CUR_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->pFctSwitchOffPwm(pHandle);
#ifdef NULL_PTR_PWR_CUR_FDB
  }
#endif
}

/**
  * @brief  Enables PWM generation on the proper Timer peripheral.
  *
  * @param  pHandle: Handler of the current instance of the PWM component.
  */
//cstat !MISRAC2012-Rule-8.13 !RED-func-no-effect
__weak void PWMC_SwitchOnPWM(PWMC_Handle_t *pHandle)
{
#ifdef NULL_PTR_PWR_CUR_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->pFctSwitchOnPwm(pHandle);
#ifdef NULL_PTR_PWR_CUR_FDB
  }
#endif
}

/**
  * @brief  Calibrates ADC current conversions by reading the offset voltage
  *         present on ADC pins when no motor current is flowing in.
  *
  * This function should be called before each motor start-up.
  *
  * @param  pHandle: Handler of the current instance of the PWM component.
  * @param  action: Can be #CRC_START to initialize the offset calibration or
  *         #CRC_EXEC to execute the offset calibration.
  * @retval true if the current calibration has been completed, **false** if it is
  *         still ongoing.
  */
__weak bool PWMC_CurrentReadingCalibr(PWMC_Handle_t *pHandle, CRCAction_t action)
{
  bool retVal = false;
#ifdef NULL_PTR_PWR_CUR_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    if (CRC_START == action)
    {
      PWMC_SwitchOffPWM(pHandle);
      pHandle->pFctCurrReadingCalib(pHandle);
      retVal = true;
    }
    else if (CRC_EXEC == action)
    {
      if (pHandle->OffCalibrWaitTimeCounter > 0u)
      {
        pHandle->OffCalibrWaitTimeCounter--;
        if (0U == pHandle->OffCalibrWaitTimeCounter)
        {
          pHandle->pFctCurrReadingCalib(pHandle);
          retVal = true;
        }
      }
      else
      {
        retVal = true;
      }
    }
    else
    {
      /* Nothing to do */
    }
#ifdef NULL_PTR_PWR_CUR_FDB
  }
#endif
  return (retVal);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  Sets a low pass filter.
  *
  * This function is called for setting low pass filter on Iq and Id, before getting
  * transformed in Ia and Ib by the Reverse Park function.
  *
  * @param in: Value needing to be passed through the filter (Iq and Id).
  * @param out_buf: LPF buffer.
  * @param t: Low pass filter constant.
  * @retval New value after the low pass filter.
  */
static inline int32_t PWMC_LowPassFilter(int32_t in, int32_t *out_buf, int32_t t)
{
  int32_t x;
#ifdef NULL_PTR_PWR_CUR_FDB
  if (MC_NULL == out_buf)
  {
    x = 0;
  }
  else
  {
#endif
#ifndef FULL_MISRA_C_COMPLIANCY_PWM_CURR
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
    *out_buf = (*out_buf) + ((in - ((*out_buf) >> 15)) * t);
    x = (*out_buf) >> 15; //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
#else
    *out_buf = (*out_buf) + ((in - ((*out_buf) / 32768)) * t);
    x = (*out_buf) / 32768;

#endif
#ifdef NULL_PTR_PWR_CUR_FDB
  }
#endif
  return (x);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  Converts input currents components Iqd into estimated
  *         currents Ia, Ib and Ic.
  *
  * @param  pHandle: Handler of the current instance of the PWM component.
  * @param  Iqd: Structure that will receive Iq and Id currents.
  * @param  hElAngledpp: Electrical angle.
  */
void PWMC_CalcPhaseCurrentsEst(PWMC_Handle_t *pHandle, qd_t Iqd, int16_t hElAngledpp)
{
#ifdef NULL_PTR_PWR_CUR_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    qd_t idq_ave;
    alphabeta_t ialpha_beta;
    int32_t temp1, temp2;

    idq_ave.q = (int16_t)PWMC_LowPassFilter(Iqd.q, &(pHandle->LPFIqBuf), pHandle->LPFIqd_const);
    idq_ave.d = (int16_t)PWMC_LowPassFilter(Iqd.d, &(pHandle->LPFIdBuf), pHandle->LPFIqd_const);

    ialpha_beta = MCM_Rev_Park(idq_ave, hElAngledpp);

    /* reverse Clarke */

    /*Ia*/
    pHandle->IaEst = ialpha_beta.alpha;

    temp1 = - ialpha_beta.alpha;
#ifndef FULL_MISRA_C_COMPLIANCY_PWM_CURR
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
    temp2 = (int32_t)(ialpha_beta.beta) * ((int32_t)SQRT3FACTOR >> 15);
#else
    temp2 = (int32_t)(ialpha_beta.beta) * (int32_t)SQRT3FACTOR / 32768;
#endif

    /*Ib*/
    pHandle->IbEst = (int16_t)(temp1 - temp2)/2;

    /*Ic*/
    pHandle->IcEst = (int16_t)(temp1 + temp2)/2;
#ifdef NULL_PTR_PWR_CUR_FDB
  }
#endif
}

/**
  * @brief  Switches power stage Low Sides transistors on.
  *
  * This function is meant for charging boot capacitors of the driving
  * section. It has to be called on each motor start-up when using high
  * voltage drivers.
  *
  * @param  pHandle: Handler of the current instance of the PWM component.
  * @param  ticks: Timer ticks value to be applied.
  *                Min value: 0 (low sides ON)
  *                Max value: PWM_PERIOD_CYCLES/2 (low sides OFF)
  */
//cstat !MISRAC2012-Rule-8.13 !RED-func-no-effect
__weak void PWMC_TurnOnLowSides(PWMC_Handle_t *pHandle, uint32_t ticks)
{
#ifdef NULL_PTR_PWR_CUR_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->pFctTurnOnLowSides(pHandle, ticks);
#ifdef NULL_PTR_PWR_CUR_FDB
  }
#endif
}

/** @brief  Checks if an overcurrent has occured since the last call to this function.
  *
  *	@param  pHandle: Handler of the current instance of the PWM component.
  * @retval #MC_BREAK_IN if an overcurrent has occurred since last call,
  *         and #MC_NO_FAULTS otherwise.
  */
__weak uint16_t PWMC_CheckOverCurrent(PWMC_Handle_t *pHandle) //cstat !MISRAC2012-Rule-8.13
{
#ifdef NULL_PTR_PWR_CUR_FDB
  return ((MC_NULL == pHandle) ? MC_NO_FAULTS : (uint16_t)pHandle->pFctIsOverCurrentOccurred(pHandle));
#else
  return ((uint16_t)pHandle->pFctIsOverCurrentOccurred(pHandle));
#endif
}

/**
  * @brief  Sets the over current threshold through the DAC reference voltage.
  *
  * @param  pHandle:  Handler of the current instance of the PWM component.
  * @param  hDACVref: Value of DAC reference voltage to be applied expressed as a 16bit unsigned integer.
  *					  Min value: 0 (0 V)
  *                	  Max value: 65536 (VDD_DAC)
  */
//cstat !MISRAC2012-Rule-8.13 !RED-func-no-effect
__weak void PWMC_OCPSetReferenceVoltage(PWMC_Handle_t *pHandle, uint16_t hDACVref)
{
#ifdef NULL_PTR_PWR_CUR_FDB
  if ((MC_NULL == pHandle) || (MC_NULL == pHandle->pFctOCPSetReferenceVoltage))
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->pFctOCPSetReferenceVoltage(pHandle, hDACVref);
#ifdef NULL_PTR_PWR_CUR_FDB
  }
#endif
}

/**
  * @brief  Retrieves the satus of TurnOnLowSides action.
  *
  * @param  pHandle: Handler of the current instance of the PWMC component.
  * @retval bool State of TurnOnLowSides action:
  *         **true** if TurnOnLowSides action is active, **false** otherwise.
  */
__weak bool PWMC_GetTurnOnLowSidesAction(const PWMC_Handle_t *pHandle)
{
#ifdef NULL_PTR_PWR_CUR_FDB
  return ((MC_NULL == pHandle) ? false : pHandle->TurnOnLowSidesAction);
#else
  return (pHandle->TurnOnLowSidesAction);
#endif
}

/** @brief Enables Discontinuous PWM mode using the @p pHandle PWMC component.
  *
  */
__weak void PWMC_DPWM_ModeEnable( PWMC_Handle_t * pHandle )
{
#ifdef NULL_PTR_PWR_CUR_FDB
  if (MC_NULL ==  pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->DPWM_Mode = true;
#ifdef NULL_PTR_PWR_CUR_FDB
  }
#endif
}

/** @brief Disables Discontinuous PWM mode using the @p pHandle PWMC component.
  *
  */
__weak void PWMC_DPWM_ModeDisable( PWMC_Handle_t * pHandle )
{
#ifdef NULL_PTR_PWR_CUR_FDB
  if (MC_NULL ==  pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->DPWM_Mode = false;
#ifdef NULL_PTR_PWR_CUR_FDB
  }
#endif
}

/** @brief  Returns the status of the Discontinuous PWM Mode stored in the @p pHandle PWMC component.
  *
  * @retval true if DPWM Mode is enabled, **false** otherwise.
  */
__weak bool PWMC_GetDPWM_Mode( PWMC_Handle_t * pHandle )
{
#ifdef NULL_PTR_PWR_CUR_FDB
  return ((MC_NULL == pHandle) ? false : pHandle->DPWM_Mode);
#else
  return (pHandle->DPWM_Mode);
#endif
}

/** @brief  Enables the RL detection mode by calling the function in @p pHandle PWMC component.
  *
  */
//cstat !MISRAC2012-Rule-8.13 !RED-func-no-effect
__weak void PWMC_RLDetectionModeEnable(PWMC_Handle_t *pHandle)
{
#ifdef NULL_PTR_PWR_CUR_FDB
  if ((MC_NULL == pHandle) || ( MC_NULL == pHandle->pFctRLDetectionModeEnable))
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->pFctRLDetectionModeEnable(pHandle);
#ifdef NULL_PTR_PWR_CUR_FDB
  }
#endif
}

/** @brief  Disables the RL detection mode by calling the function in @p pHandle PWMC component.
  *
  */
//cstat !MISRAC2012-Rule-8.13 !RED-func-no-effect
__weak void PWMC_RLDetectionModeDisable(PWMC_Handle_t *pHandle)
{
#ifdef NULL_PTR_PWR_CUR_FDB
  if ((MC_NULL == pHandle) || ( MC_NULL == pHandle->pFctRLDetectionModeDisable))
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->pFctRLDetectionModeDisable(pHandle);
#ifdef NULL_PTR_PWR_CUR_FDB
  }
#endif
}

/**
  * @brief  Sets the PWM duty cycle to apply in the RL Detection mode.
  *
  * @param  pHandle: Handler of the current instance of the PWMC component.
  * @param  hDuty: Duty cycle to apply, written in uint16_t.
  * @retval #MC_NO_ERROR if the Duty Cycle could be applied on time for the next PWM period.
  * 		Returns #MC_DURATION otherwise.
  */
__weak uint16_t PWMC_RLDetectionModeSetDuty(PWMC_Handle_t *pHandle, uint16_t hDuty) //cstat !MISRAC2012-Rule-8.13
{
#ifdef NULL_PTR_PWR_CUR_FDB
 uint16_t retVal = MC_DURATION;

 if ((MC_NULL == pHandle) || ( MC_NULL == pHandle->pFctRLDetectionModeSetDuty))
 {
   /* Nothing to do */
 }
 else
 {
   retVal = pHandle->pFctRLDetectionModeSetDuty(pHandle, hDuty);
 }
 return (retVal);
#else
  return (pHandle->pFctRLDetectionModeSetDuty(pHandle, hDuty));
#endif
}

/**
  * @brief  Sets the aligned motor flag.
  *
  * @param  pHandle: Handler of the current instance of the PWMC component.
  * @param  flag: Value to be applied as an 8 bit unsigned integer.
  *				  1: motor is in aligned stage.
  *               2: motor is not in aligned stage.
  */
void PWMC_SetAlignFlag(PWMC_Handle_t *pHandle, uint8_t flag)
{
#ifdef NULL_PTR_PWR_CUR_FDB
  if (MC_NULL ==  pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->AlignFlag = flag;
#ifdef NULL_PTR_PWR_CUR_FDB
  }
#endif
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to get phases current.
 *
 * @param pCallBack: Pointer on the callback to get the phase current.
 * @param pHandle: Handler of the current instance of the PWMC component.
 */
__weak void PWMC_RegisterGetPhaseCurrentsCallBack(PWMC_GetPhaseCurr_Cb_t pCallBack, PWMC_Handle_t *pHandle)
{
#ifdef NULL_PTR_PWR_CUR_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->pFctGetPhaseCurrents = pCallBack;
#ifdef NULL_PTR_PWR_CUR_FDB
  }
#endif
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to switch PWM
 *        generation off.
 *
 * @param pCallBack: Pointer on the generic callback.
 * @param pHandle: Handler of the current instance of the PWMC component.
 */
__weak void PWMC_RegisterSwitchOffPwmCallBack(PWMC_Generic_Cb_t pCallBack, PWMC_Handle_t *pHandle)
{
#ifdef NULL_PTR_PWR_CUR_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->pFctSwitchOffPwm = pCallBack;
#ifdef NULL_PTR_PWR_CUR_FDB
  }
#endif
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to switch PWM
 *        generation on.
 *
 * @param pCallBack: Pointer on the generic callback.
 * @param pHandle: Handler of the current instance of the PWMC component.
 */
__weak void PWMC_RegisterSwitchonPwmCallBack(PWMC_Generic_Cb_t pCallBack, PWMC_Handle_t *pHandle)
{
#ifdef NULL_PTR_PWR_CUR_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->pFctSwitchOnPwm = pCallBack;
#ifdef NULL_PTR_PWR_CUR_FDB
  }
#endif
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to execute a calibration
 *        of the current sensing system.
 *
 * @param pCallBack: Pointer on the generic callback.
 * @param pHandle: Handler of the current instance of the PWMC component.
 */
__weak void PWMC_RegisterReadingCalibrationCallBack(PWMC_Generic_Cb_t pCallBack, PWMC_Handle_t *pHandle)
{
#ifdef NULL_PTR_PWR_CUR_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->pFctCurrReadingCalib = pCallBack;
#ifdef NULL_PTR_PWR_CUR_FDB
  }
#endif
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to turn low sides on.
 *
 * @param pCallBack: Pointer on the callback which turns low sides on.
 * @param pHandle: Handler of the current instance of the PWMC component.
 */
__weak void PWMC_RegisterTurnOnLowSidesCallBack(PWMC_TurnOnLowSides_Cb_t pCallBack, PWMC_Handle_t *pHandle)
{
#ifdef NULL_PTR_PWR_CUR_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->pFctTurnOnLowSides = pCallBack;
#ifdef NULL_PTR_PWR_CUR_FDB
  }
#endif
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to compute ADC sampling point.
 *
 * @param pCallBack: Pointer on the callback which sets the sampling point depending on the sector.
 * @param pHandle: Handler of the current instance of the PWMC component.
 */
__weak void PWMC_RegisterSampPointSectXCallBack(PWMC_SetSampPointSectX_Cb_t pCallBack, PWMC_Handle_t *pHandle)
{
#ifdef NULL_PTR_PWR_CUR_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->pFctSetADCSampPointSectX = pCallBack;
#ifdef NULL_PTR_PWR_CUR_FDB
  }
#endif
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to check the overcurrent status.
 *
 * @param pCallBack: Pointer on the callback which checks the overcurrent state.
 * @param pHandle: Handler of the current instance of the PWMC component.
 */
__weak void PWMC_RegisterIsOverCurrentOccurredCallBack(PWMC_OverCurr_Cb_t pCallBack, PWMC_Handle_t *pHandle)
{
#ifdef NULL_PTR_PWR_CUR_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->pFctIsOverCurrentOccurred = pCallBack;
#ifdef NULL_PTR_PWR_CUR_FDB
  }
#endif
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to set the reference
 *        voltage for the overcurrent protection.
 *
 * @param pCallBack: Pointer on the callback which sets the reference voltage.
 * @param pHandle: Handler of the current instance of the PWMC component.
 */
__weak void PWMC_RegisterOCPSetRefVoltageCallBack(PWMC_SetOcpRefVolt_Cb_t pCallBack, PWMC_Handle_t *pHandle)
{
#ifdef NULL_PTR_PWR_CUR_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->pFctOCPSetReferenceVoltage = pCallBack;
#ifdef NULL_PTR_PWR_CUR_FDB
  }
#endif
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to enable the R/L detection mode.
 *
 * @param pCallBack: Pointer on the generic callback.
 * @param pHandle: Handler of the current instance of the PWMC component.
 */
__weak void PWMC_RegisterRLDetectionModeEnableCallBack(PWMC_Generic_Cb_t pCallBack, PWMC_Handle_t *pHandle)
{
#ifdef NULL_PTR_PWR_CUR_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->pFctRLDetectionModeEnable = pCallBack;
#ifdef NULL_PTR_PWR_CUR_FDB
  }
#endif
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to disable the R/L detection mode.
 *
 * @param pCallBack: Pointer on the generic callback.
 * @param pHandle: Handler of the current instance of the PWMC component.
 */
__weak void PWMC_RegisterRLDetectionModeDisableCallBack(PWMC_Generic_Cb_t pCallBack, PWMC_Handle_t *pHandle)
{
#ifdef NULL_PTR_PWR_CUR_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->pFctRLDetectionModeDisable = pCallBack;
#ifdef NULL_PTR_PWR_CUR_FDB
  }
#endif
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to set the duty cycle
 *        for the R/L detection mode.
 *
 * @param pCallBack: Pointer on the callback which sets the duty cycle.
 * @param pHandle: Handler of the current instance of the PWMC component.
 */
__weak void PWMC_RegisterRLDetectionModeSetDutyCallBack(PWMC_RLDetectSetDuty_Cb_t pCallBack, PWMC_Handle_t *pHandle)
{
#ifdef NULL_PTR_PWR_CUR_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->pFctRLDetectionModeSetDuty = pCallBack;
#ifdef NULL_PTR_PWR_CUR_FDB
  }
#endif
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
