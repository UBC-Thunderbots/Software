/**
  ******************************************************************************
  * @file    encoder_speed_pos_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the following features
  *          of the Encoder component of the Motor Control SDK:
  *           - computes and stores average mechanical speed
  *           - computes and stores average mechanical acceleration
  *           - computes and stores  the instantaneous electrical speed
  *           - calculates the rotor electrical and mechanical angle
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
  * @ingroup Encoder
  */

/* Includes ------------------------------------------------------------------*/
#include "encoder_speed_pos_fdbk.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk
  * @{
  */

/** @defgroup Encoder Encoder Speed & Position Feedback
  * @brief Quadrature Encoder based Speed & Position Feedback implementation
  *
  * This component is used in applications controlling a motor equipped with a quadrature encoder.
  *
  * This component uses the output of a quadrature encoder to provide a measure of the speed and
  * the motor rotor position.
  *
  * More detail in [Encoder Speed & Position Feedback module](rotor_speed_pos_feedback_qenc.md).
  *
  * @{
  */

/* Private defines -----------------------------------------------------------*/


/**
  * @brief  It initializes the hardware peripherals
            required for the speed position sensor management using ENCODER
            sensors.
  * @param  pHandle: handler of the current instance of the encoder component
  */
__weak void ENC_Init(ENCODER_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
  if (NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    TIM_TypeDef *TIMx = pHandle->TIMx;
    uint8_t bufferSize;
    uint8_t index;

#ifdef TIM_CNT_UIFCPY
    LL_TIM_EnableUIFRemap(TIMx);
#define ENC_MAX_OVERFLOW_NB     ((uint16_t)2048) /* 2^11*/
#else
#define ENC_MAX_OVERFLOW_NB     (1)
#endif
    /* Reset counter */
    LL_TIM_SetCounter(TIMx, 0);

    /*Calculations of convenience*/
    pHandle->U32MAXdivPulseNumber = UINT32_MAX / ((uint32_t) pHandle->PulseNumber);
    pHandle->SpeedSamplingFreqUnit = ((uint32_t)pHandle->SpeedSamplingFreqHz * (uint32_t)SPEED_UNIT);

    /* Set IC filter for both channel 1 & 2*/
    LL_TIM_IC_SetFilter(TIMx, LL_TIM_CHANNEL_CH1, ((uint32_t)pHandle->ICx_Filter << 20U));
    LL_TIM_IC_SetFilter(TIMx, LL_TIM_CHANNEL_CH2, ((uint32_t)pHandle->ICx_Filter << 20U));

    LL_TIM_ClearFlag_UPDATE(TIMx);
    LL_TIM_EnableIT_UPDATE(TIMx);

    /* Enable the counting timer*/
    LL_TIM_EnableCounter(TIMx);

    /* Erase speed buffer */
    bufferSize = pHandle->SpeedBufferSize;

    for (index = 0U; index < bufferSize; index++)
    {
      pHandle->DeltaCapturesBuffer[index] = 0;
    }
#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  Clear software FIFO where the captured rotor angle variations are stored.
  *         This function must be called before starting the motor to initialize
  *         the speed measurement process.
  * @param  pHandle: handler of the current instance of the encoder component
  */
__weak void ENC_Clear(ENCODER_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
  if (NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    uint8_t index;

    for (index = 0u; index < pHandle->SpeedBufferSize; index++)
    {
      pHandle->DeltaCapturesBuffer[index] = 0;
    }
    pHandle->SensorIsReliable = true;
#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  It calculates the rotor electrical and mechanical angle, on the basis
  *         of the instantaneous value of the timer counter.
  * @param  pHandle: handler of the current instance of the encoder component
  * @retval Measured electrical angle in [s16degree](measurement_units.md) format.
  */
__weak int16_t ENC_CalcAngle(ENCODER_Handle_t *pHandle)
{
  int16_t elAngle;  /* s16degree format */
#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
  if (NULL == pHandle)
  {
    elAngle = 0;
  }
  else
  {
#endif
    int16_t mecAngle; /* s16degree format */
    uint32_t uwtemp1;
    int32_t wtemp1;
    /* PR 52926 We need to keep only the 16 LSB, bit 31 could be at 1
     if the overflow occurs just after the entry in the High frequency task */
    uwtemp1 = (LL_TIM_GetCounter(pHandle->TIMx) & 0xffffU) * (pHandle->U32MAXdivPulseNumber);
#ifndef FULL_MISRA_C_COMPLIANCY_ENC_SPD_POS
    wtemp1 = (int32_t)uwtemp1 >> 16U;  //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
#else
    wtemp1 = (int32_t)uwtemp1 / 65536;
#endif
    /*Computes and stores the rotor mechanical angle*/
    mecAngle = (int16_t)wtemp1;

    int16_t hMecAnglePrev = pHandle->_Super.hMecAngle;

    pHandle->_Super.hMecAngle = mecAngle;

    /*Computes and stores the rotor electrical angle*/
    elAngle = mecAngle * (int16_t)(pHandle->_Super.bElToMecRatio);

    pHandle->_Super.hElAngle = elAngle;

    int16_t hMecSpeedDpp = mecAngle - hMecAnglePrev;
    pHandle->_Super.wMecAngle += ((int32_t)hMecSpeedDpp);
#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
  }
#endif
  /*Returns rotor electrical angle*/
  return (elAngle);
}

/**
  * @brief  This method must be called with the periodicity defined by parameter
  *         SpeedSamplingFreqUnit. The method generates a capture event on
  *         a channel, computes and stores average mechanical speed expressed in the unit
  *         defined by #SPEED_UNIT (on the basis of the buffer filled by Medium Frequency Task),
  *         computes and stores average mechanical acceleration expressed in #SPEED_UNIT/SpeedSamplingFreq,
  *         computes and stores the instantaneous electrical speed in Digit Per control Period
  *         unit [dpp](measurement_units.md), updates the index of the
  *         speed buffer, then checks, stores and returns the reliability state
  *         of the sensor.
  * @param  pHandle: handler of the current instance of the encoder component
  * @param  pMecSpeedUnit pointer used to return the rotor average mechanical speed
  *         expressed in the unit defined by #SPEED_UNIT
  * @retval true = sensor information is reliable. false = sensor information is not reliable
  */
__weak bool ENC_CalcAvrgMecSpeedUnit(ENCODER_Handle_t *pHandle, int16_t *pMecSpeedUnit)
{
  bool bReliability;
#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
  if ((NULL == pHandle) || (NULL == pMecSpeedUnit))
  {
    bReliability = false;
  }
  else
  {
#endif
    int32_t wtemp1;
    int32_t wtemp2;
    uint32_t OverflowCntSample;
    uint32_t CntCapture;
    uint32_t directionSample;
    int32_t wOverallAngleVariation = 0;
    TIM_TypeDef *TIMx = pHandle->TIMx;
    uint8_t bBufferSize = pHandle->SpeedBufferSize;
    uint8_t bBufferIndex;
#ifdef TIM_CNT_UIFCPY
    uint8_t OFbit;
#else
    uint8_t OFbit = 0U;
#endif

#ifdef TIM_CNT_UIFCPY
    /* disable Interrupt generation */
    LL_TIM_DisableIT_UPDATE(TIMx);
#endif
    CntCapture = LL_TIM_GetCounter(TIMx);
    OverflowCntSample = pHandle->TimerOverflowNb;
    pHandle->TimerOverflowNb = 0;
    directionSample = LL_TIM_GetDirection(TIMx);
#ifdef TIM_CNT_UIFCPY
    OFbit = __LL_TIM_GETFLAG_UIFCPY(CntCapture);
    if (0U == OFbit)
    {
      /* Nothing to do */
    }
    else
    {
      /* If OFbit is set, overflow has occured since IT has been disabled.
      We have to take this overflow into account in the angle computation,
      but we must not take it into account a second time in the accmulator,
      so we have to clear the pending flag. If the OFbit is not set, it does not mean
      that an Interrupt has not occured since the last read, but it has not been taken
      into accout, we must not clear the interrupt in order to accumulate it */
      LL_TIM_ClearFlag_UPDATE(TIMx);
    }

    LL_TIM_EnableIT_UPDATE(TIMx);
    CLEAR_BIT(CntCapture, TIM_CNT_UIFCPY);
#endif

    /* If UIFCPY is not present, OverflowCntSample can not be used safely for
    speed computation, but we still use it to check that we do not exceed one overflow
    (sample frequency not less than mechanical motor speed */

    if ((OverflowCntSample + OFbit) > ENC_MAX_OVERFLOW_NB)
    {
      pHandle->TimerOverflowError = true;
    }
    else
    {
      /* Nothing to do */
    }

    /*Calculation of delta angle*/
    if (LL_TIM_COUNTERDIRECTION_DOWN == directionSample)
    {
      /* encoder timer down-counting*/
      /* if UIFCPY not present Overflow counter can not be safely used -> limitation to 1 OF. */
#ifndef TIM_CNT_UIFCPY
      OverflowCntSample = (CntCapture > pHandle->PreviousCapture) ? 1 : 0;
#endif
      pHandle->DeltaCapturesBuffer[pHandle->DeltaCapturesIndex] =
        ((int32_t)CntCapture) - ((int32_t)pHandle->PreviousCapture)
        - ((((int32_t)OverflowCntSample) + (int32_t)OFbit) * ((int32_t)pHandle->PulseNumber));
    }
    else
    {
      /* encoder timer up-counting*/
      /* if UIFCPY not present Overflow counter can not be safely used -> limitation to 1 OF. */
#ifndef TIM_CNT_UIFCPY
      OverflowCntSample = (CntCapture < pHandle->PreviousCapture) ? 1 : 0;
#endif
      pHandle->DeltaCapturesBuffer[pHandle->DeltaCapturesIndex] =
        ((int32_t)CntCapture) - ((int32_t)pHandle->PreviousCapture)
        + ((((int32_t)OverflowCntSample) + (int32_t)OFbit) * ((int32_t)pHandle->PulseNumber));
    }


    /*Computes & returns average mechanical speed */
    for (bBufferIndex = 0U; bBufferIndex < bBufferSize; bBufferIndex++)
    {
      wOverallAngleVariation += pHandle->DeltaCapturesBuffer[bBufferIndex];
    }
    wtemp1 = wOverallAngleVariation * ((int32_t)pHandle->SpeedSamplingFreqUnit);
    wtemp2 = ((int32_t)pHandle->PulseNumber) * ((int32_t)pHandle->SpeedBufferSize);
    wtemp1 = ((0 == wtemp2) ? wtemp1 : (wtemp1 / wtemp2));

    *pMecSpeedUnit = (int16_t)wtemp1;

    /*Computes & stores average mechanical acceleration */
    pHandle->_Super.hMecAccelUnitP = (int16_t)(wtemp1 - pHandle->_Super.hAvrMecSpeedUnit);

    /*Stores average mechanical speed */
    pHandle->_Super.hAvrMecSpeedUnit = (int16_t)wtemp1;

    /*Computes & stores the instantaneous electrical speed [dpp], var wtemp1*/
    wtemp1 = pHandle->DeltaCapturesBuffer[pHandle->DeltaCapturesIndex] * ((int32_t)pHandle->SpeedSamplingFreqHz)
             * ((int32_t)pHandle->_Super.bElToMecRatio);
    wtemp1 /= ((int32_t)pHandle->PulseNumber);
    wtemp1 *= ((int32_t)pHandle->_Super.DPPConvFactor);
    wtemp1 /= ((int32_t)pHandle->_Super.hMeasurementFrequency);

    pHandle->_Super.hElSpeedDpp = (int16_t)wtemp1;

    /*last captured value update*/
    pHandle->PreviousCapture = (CntCapture >= (uint32_t)65535) ? 65535U : (uint16_t)CntCapture;
    /*Buffer index update*/
    pHandle->DeltaCapturesIndex++;

    if (pHandle->DeltaCapturesIndex >= pHandle->SpeedBufferSize)
    {
      pHandle->DeltaCapturesIndex = 0U;
    }
    else
    {
      /* nothing to do */
    }

    /*Checks the reliability status, then stores and returns it*/
    if (pHandle->TimerOverflowError)
    {
      bReliability = false;
      pHandle->SensorIsReliable = false;
      pHandle->_Super.bSpeedErrorNumber = pHandle->_Super.bMaximumSpeedErrorsNumber;
    }
    else
    {
      bReliability = SPD_IsMecSpeedReliable(&pHandle->_Super, pMecSpeedUnit);
    }
#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
  }
#endif
  return (bReliability);
}

/**
  * @brief  It set instantaneous rotor mechanical angle.
  *         As a consequence, timer counter is computed and updated.
  * @param  pHandle: handler of the current instance of the encoder component
  * @param  hMecAngle new value of rotor mechanical angle in [s16degree](measurement_units.md) format.
  */
__weak void ENC_SetMecAngle(ENCODER_Handle_t *pHandle, int16_t hMecAngle)
{
#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
  if (NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    TIM_TypeDef *TIMx = pHandle->TIMx;

    uint16_t hAngleCounts;
    uint16_t hMecAngleuint;
    int16_t localhMecAngle = hMecAngle;

    pHandle->_Super.hMecAngle = localhMecAngle;
    pHandle->_Super.hElAngle = localhMecAngle * (int16_t)pHandle->_Super.bElToMecRatio;
    if (localhMecAngle < 0)
    {
      localhMecAngle *= -1;
      hMecAngleuint = 65535U - ((uint16_t)localhMecAngle);
    }
    else
    {
      hMecAngleuint = (uint16_t)localhMecAngle;
    }

    hAngleCounts = (uint16_t)((((uint32_t)hMecAngleuint) * ((uint32_t)pHandle->PulseNumber)) / 65535U);

    TIMx->CNT = (uint16_t)hAngleCounts;
#ifdef NULL_PTR_CHECK_ENC_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  TIMER ENCODER Overflow interrupt counter update
  * @param  pHandleVoid: handler of the current instance of the encoder component
  */
__weak void *ENC_IRQHandler(void *pHandleVoid)
{
  ENCODER_Handle_t *pHandle = (ENCODER_Handle_t *)pHandleVoid; //cstat !MISRAC2012-Rule-11.5

  /*Updates the number of overflows occurred*/
  /* the handling of overflow error is done in ENC_CalcAvrgMecSpeedUnit */
  pHandle->TimerOverflowNb += 1U;

  return (MC_NULL);
}
/**
  * @}
  */

/**
  * @}
  */

/** @} */


/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
