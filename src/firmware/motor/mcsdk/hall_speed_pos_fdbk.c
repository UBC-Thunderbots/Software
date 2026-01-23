
/**
 ******************************************************************************
 * @file    hall_speed_pos_fdbk.c
 * @author  Motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the features of
 *          the Hall Speed & Position Feedback component of the Motor Control SDK.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 * @ingroup hall_speed_pos_fdbk
 */

/* Includes ------------------------------------------------------------------*/
#include "firmware/motor/mcsdk/hall_speed_pos_fdbk.h"

#include "firmware/motor/common_defs.h"
#include "firmware/motor/mcsdk/mc_type.h"
#include "firmware/motor/mcsdk/speed_pos_fdbk.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup SpeednPosFdbk
 * @{
 */

/** @addtogroup hall_speed_pos_fdbk
 * @{
 */

/**
 * @defgroup hall_speed_pos_fdbk Hall Speed & Position Feedback
 *
 * @brief Hall Sensor based Speed & Position Feedback implementation
 *
 * This component is used in applications controlling a motor equipped with Hall effect
 * sensors.
 *
 * This component uses the output of two Hall effects sensors to provide a measure of the
 * speed and the position of the rotor of the motor.
 *
 * @todo Document the Hall Speed & Position Feedback "module".
 *
 * @{
 */

/* Private defines -----------------------------------------------------------*/

/* Lower threshold to reques a decrease of clock prescaler */
#define LOW_RES_THRESHOLD ((uint16_t)0x5500U)

#define HALL_COUNTER_RESET ((uint16_t)0U)

#define S16_120_PHASE_SHIFT (int16_t)(65536 / 3)
#define S16_60_PHASE_SHIFT (int16_t)(65536 / 6)

#define STATE_0 (uint8_t)0
#define STATE_1 (uint8_t)1
#define STATE_2 (uint8_t)2
#define STATE_3 (uint8_t)3
#define STATE_4 (uint8_t)4
#define STATE_5 (uint8_t)5
#define STATE_6 (uint8_t)6
#define STATE_7 (uint8_t)7

#define NEGATIVE (int8_t) - 1
#define POSITIVE (int8_t)1

/* With digit-per-PWM unit (here 2*PI rad = 0xFFFF): */
#define HALL_MAX_PSEUDO_SPEED ((int16_t)0x7FFF)

#define CCER_CC1E_Set ((uint16_t)0x0001)
#define CCER_CC1E_Reset ((uint16_t)0xFFFE)

static void HALL_Init_Electrical_Angle(HALL_Handle_t *pHandle);

/**
  * @brief  It initializes the hardware peripherals (TIMx, GPIO and NVIC)
            required for the speed position sensor management using HALL
            sensors.
  * @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
  */
__weak void HALL_Init(HALL_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_HALL_SPD_POS_FDB
    if (NULL == pHandle)
    {
        /* Nothing to do */
    }
    else
    {
#endif
        TIM_TypeDef *TIMx = pHandle->TIMx;

        uint16_t hMinReliableElSpeedUnit =
            pHandle->_Super.hMinReliableMecSpeedUnit * pHandle->_Super.bElToMecRatio;
        uint16_t hMaxReliableElSpeedUnit =
            pHandle->_Super.hMaxReliableMecSpeedUnit * pHandle->_Super.bElToMecRatio;
        uint8_t bSpeedBufferSize;
        uint8_t bIndex;

        /* Adjustment factor: minimum measurable speed is x time less than the minimum
        reliable speed */
        hMinReliableElSpeedUnit /= 4U;

        /* Adjustment factor: maximum measurable speed is x time greater than the
        maximum reliable speed */
        hMaxReliableElSpeedUnit *= 2U;

        pHandle->OvfFreq = (uint16_t)(pHandle->TIMClockFreq / 65536U);

        /* SW Init */
        if (0U == hMinReliableElSpeedUnit)
        {
            /* Set fixed to 150 ms */
            pHandle->HallTimeout = 150U;
        }
        else
        {
            /* Set accordingly the min reliable speed */
            /* 1000 comes from mS
             * 6 comes from the fact that sensors are toggling each 60 deg = 360/6 deg */
            pHandle->HallTimeout =
                (1000U * (uint16_t)SPEED_UNIT) / (6U * hMinReliableElSpeedUnit);
        }

        /* Compute the prescaler to the closet value of the TimeOut (in mS )*/
        pHandle->HALLMaxRatio = (pHandle->HallTimeout * pHandle->OvfFreq) / 1000U;

        /* Align MaxPeriod to a multiple of Overflow.*/
        pHandle->MaxPeriod = pHandle->HALLMaxRatio * 65536UL;

        pHandle->SatSpeed = hMaxReliableElSpeedUnit;

        pHandle->PseudoFreqConv =
            ((pHandle->TIMClockFreq / 6U) / pHandle->_Super.hMeasurementFrequency) *
            pHandle->_Super.DPPConvFactor;

        if (0U == hMaxReliableElSpeedUnit)
        {
            pHandle->MinPeriod = ((uint32_t)SPEED_UNIT * (pHandle->TIMClockFreq / 6UL));
        }
        else
        {
            pHandle->MinPeriod = (((uint32_t)SPEED_UNIT * (pHandle->TIMClockFreq / 6UL)) /
                                  hMaxReliableElSpeedUnit);
        }

        pHandle->PWMNbrPSamplingFreq =
            ((pHandle->_Super.hMeasurementFrequency * pHandle->PWMFreqScaling) /
             pHandle->SpeedSamplingFreqHz) -
            1U;

        /* Reset speed reliability */
        pHandle->SensorIsReliable = true;

        /* Set IC filter for Channel 1 (ICF1) */
        LL_TIM_IC_SetFilter(TIMx, LL_TIM_CHANNEL_CH1, (uint32_t)(pHandle->ICx_Filter));

        /* Force the TIMx prescaler with immediate access (gen update event)
         */
        LL_TIM_SetPrescaler(TIMx, pHandle->HALLMaxRatio);
        LL_TIM_GenerateEvent_UPDATE(TIMx);

        /* Clear the TIMx's pending flags */
        WRITE_REG(TIMx->SR, 0);

        /* Selected input capture and Update (overflow) events generate interrupt */

        /* Source of Update event is only counter overflow/underflow */
        LL_TIM_SetUpdateSource(TIMx, LL_TIM_UPDATESOURCE_COUNTER);

        LL_TIM_EnableIT_CC1(TIMx);
        LL_TIM_EnableIT_UPDATE(TIMx);
        LL_TIM_SetCounter(TIMx, HALL_COUNTER_RESET);

        LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1);
        LL_TIM_EnableCounter(TIMx);

        /* Erase speed buffer */
        bSpeedBufferSize = pHandle->SpeedBufferSize;

        for (bIndex = 0u; bIndex < bSpeedBufferSize; bIndex++)
        {
            pHandle->SensorPeriod[bIndex] = (int32_t)pHandle->MaxPeriod;
        }
#ifdef NULL_PTR_CHECK_HALL_SPD_POS_FDB
    }
#endif
}

/**
 * @brief  Clear software FIFO where are "pushed" latest speed information
 *         This function must be called before starting the motor to initialize
 *         the speed measurement process.
 * @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component*
 */
__weak void HALL_Clear(HALL_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_HALL_SPD_POS_FDB
    if (NULL == pHandle)
    {
        /* Nothing to do */
    }
    else
    {
#endif
        TIM_TypeDef *TIMx = pHandle->TIMx;

        /* Mask interrupts to insure a clean intialization */
        LL_TIM_DisableIT_CC1(TIMx);

        pHandle->RatioDec = false;
        pHandle->RatioInc = false;

        /* Reset speed reliability */
        pHandle->SensorIsReliable = true;

        /* Acceleration measurement not implemented.*/
        pHandle->_Super.hMecAccelUnitP = 0;

        pHandle->FirstCapt    = 0U;
        pHandle->BufferFilled = 0U;
        pHandle->OVFCounter   = 0U;

        pHandle->CompSpeed = 0;

        pHandle->Direction = POSITIVE;

        /* Initialize speed buffer index */
        pHandle->SpeedFIFOIdx = 0U;

        /* Clear speed error counter */
        pHandle->_Super.bSpeedErrorNumber = 0;

        /* Re-initialize partly the timer */
        LL_TIM_SetPrescaler(TIMx, pHandle->HALLMaxRatio);

        LL_TIM_SetCounter(TIMx, HALL_COUNTER_RESET);

        LL_TIM_EnableCounter(TIMx);

        LL_TIM_EnableIT_CC1(TIMx);

        HALL_Init_Electrical_Angle(pHandle);
#ifdef NULL_PTR_CHECK_HALL_SPD_POS_FDB
    }
#endif
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  Update the rotor electrical angle integrating the last measured
 *         instantaneous electrical speed express in dpp.
 * @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
 * @retval int16_t Measured electrical angle in s16degree format.
 */
__weak int16_t HALL_CalcElAngle(HALL_Handle_t *pHandle)
{
    int16_t retValue;
#ifdef NULL_PTR_CHECK_HALL_SPD_POS_FDB
    if (NULL == pHandle)
    {
        retValue = 0;
    }
    else
    {
#endif

        if (pHandle->_Super.hElSpeedDpp != HALL_MAX_PSEUDO_SPEED)
        {
            pHandle->IncrementElAngle += pHandle->_Super.hElSpeedDpp + pHandle->CompSpeed;
            pHandle->PrevRotorFreq = pHandle->_Super.hElSpeedDpp;

            if (pHandle->IncrementElAngle >= S16_60_PHASE_SHIFT)
            {
                pHandle->_Super.hElAngle +=
                    pHandle->_Super.hElSpeedDpp + pHandle->CompSpeed -
                    (pHandle->IncrementElAngle - S16_60_PHASE_SHIFT) - 1;
                pHandle->IncrementElAngle = S16_60_PHASE_SHIFT;
            }
            else if (pHandle->IncrementElAngle <= -S16_60_PHASE_SHIFT)
            {
                pHandle->_Super.hElAngle +=
                    pHandle->_Super.hElSpeedDpp + pHandle->CompSpeed -
                    (pHandle->IncrementElAngle + S16_60_PHASE_SHIFT) + 1;
                pHandle->IncrementElAngle = -S16_60_PHASE_SHIFT;
            }
            else
            {
                pHandle->MeasuredElAngle += pHandle->_Super.hElSpeedDpp;
                pHandle->_Super.hElAngle +=
                    pHandle->_Super.hElSpeedDpp + pHandle->CompSpeed;
            }
        }
        else
        {
            pHandle->_Super.hElAngle += pHandle->PrevRotorFreq;
        }

        retValue = pHandle->_Super.hElAngle;
#ifdef NULL_PTR_CHECK_HALL_SPD_POS_FDB
    }
#endif
    return (retValue);
}

/**
 * @brief  This method must be called - at least - with the same periodicity
 *         on which speed control is executed.
 *         This method compute and store rotor istantaneous el speed (express
 *         in dpp considering the measurement frequency) in order to provide it
 *         to HALL_CalcElAngle function and SPD_GetElAngle.
 *         Then compute rotor average el speed (express in dpp considering the
 *         measurement frequency) based on the buffer filled by IRQ, then - as
 *         a consequence - compute, store and return - through parameter
 *         hMecSpeedUnit - the rotor average mech speed, expressed in Unit.
 *         Then check, store and return the reliability state of
 *         the sensor; in this function the reliability is measured with
 *         reference to specific parameters of the derived
 *         sensor (HALL) through internal variables managed by IRQ.
 * @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
 * @param  hMecSpeedUnit pointer to int16_t, used to return the rotor average
 *         mechanical speed (expressed in the unit defined by #SPEED_UNIT)
 * @retval true = sensor information is reliable
 *         false = sensor information is not reliable
 */
__weak bool HALL_CalcAvrgMecSpeedUnit(HALL_Handle_t *pHandle, int16_t *hMecSpeedUnit)
{
    bool bReliability;
#ifdef NULL_PTR_CHECK_HALL_SPD_POS_FDB
    if ((NULL == pHandle) || (NULL == hMecSpeedUnit))
    {
        bReliability = false;
    }
    else
    {
#endif
        TIM_TypeDef *TIMx = pHandle->TIMx;

        if (pHandle->SensorIsReliable)
        {
            /* No errors have been detected during rotor speed information
            extrapolation */
            if (LL_TIM_GetPrescaler(TIMx) >= pHandle->HALLMaxRatio)
            {
                /* At start-up or very low freq */
                /* Based on current prescaler value only */
                pHandle->_Super.hElSpeedDpp = 0;
                *hMecSpeedUnit              = 0;
            }
            else
            {
                pHandle->_Super.hElSpeedDpp = pHandle->AvrElSpeedDpp;
                if (0 == pHandle->AvrElSpeedDpp)
                {
                    /* Speed is too low */
                    *hMecSpeedUnit = 0;
                }
                else
                {
                    /* Check if speed is not to fast */
                    if (pHandle->AvrElSpeedDpp != HALL_MAX_PSEUDO_SPEED)
                    {
                        if (true == pHandle->HallMtpa)
                        {
                            pHandle->CompSpeed = 0;
                        }
                        else
                        {
                            pHandle->DeltaAngle =
                                pHandle->MeasuredElAngle - pHandle->_Super.hElAngle;
                            pHandle->CompSpeed =
                                (int16_t)((int32_t)(pHandle->DeltaAngle) /
                                          (int32_t)(pHandle->PWMNbrPSamplingFreq));
                        }
                        /* Convert el_dpp to MecUnit */
                        *hMecSpeedUnit =
                            (int16_t)((pHandle->AvrElSpeedDpp *
                                       (int32_t)pHandle->_Super.hMeasurementFrequency *
                                       (int32_t)SPEED_UNIT) /
                                      ((int32_t)(pHandle->_Super.DPPConvFactor) *
                                       (int32_t)(pHandle->_Super.bElToMecRatio)));
                    }
                    else
                    {
                        *hMecSpeedUnit = (int16_t)pHandle->SatSpeed;
                    }
                }
            }
            bReliability = SPD_IsMecSpeedReliable(&pHandle->_Super, hMecSpeedUnit);
        }
        else
        {
            bReliability                      = false;
            pHandle->_Super.bSpeedErrorNumber = pHandle->_Super.bMaximumSpeedErrorsNumber;
            /* If speed is not reliable the El and Mec speed is set to 0 */
            pHandle->_Super.hElSpeedDpp = 0;
            *hMecSpeedUnit              = 0;
        }

        pHandle->_Super.hAvrMecSpeedUnit = *hMecSpeedUnit;
#ifdef NULL_PTR_CHECK_HALL_SPD_POS_FDB
    }
#endif

    return (bReliability);
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  Example of private method of the class HALL to implement an MC IRQ function
 *         to be called when TIMx capture event occurs
 * @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
 */
__weak void *HALL_TIMx_CC_IRQHandler(void *pHandleVoid)
{
    uint32_t wCaptBuf;
    uint16_t hPrscBuf;
    uint16_t hHighSpeedCapture;
    HALL_Handle_t *pHandle = (HALL_Handle_t *)pHandleVoid;  // cstat !MISRAC2012-Rule-11.5
    TIM_TypeDef *TIMx      = pHandle->TIMx;
    uint8_t bPrevHallState;
    int8_t PrevDirection;

    if (pHandle->SensorIsReliable)
    {
        /* A capture event generated this interrupt */
        bPrevHallState = pHandle->HallState;
        PrevDirection  = pHandle->Direction;

        if (DEGREES_120 == pHandle->SensorPlacement)
        {
            pHandle->HallState =
                (uint8_t)((LL_GPIO_IsInputPinSet(pHandle->H3Port, pHandle->H3Pin) << 2U) |
                          (LL_GPIO_IsInputPinSet(pHandle->H2Port, pHandle->H2Pin) << 1U) |
                          LL_GPIO_IsInputPinSet(pHandle->H1Port, pHandle->H1Pin));
        }
        else
        {
            pHandle->HallState =
                (uint8_t)(((LL_GPIO_IsInputPinSet(pHandle->H2Port, pHandle->H2Pin) ^ 1U)
                           << 2U) |
                          (LL_GPIO_IsInputPinSet(pHandle->H3Port, pHandle->H3Pin) << 1U) |
                          LL_GPIO_IsInputPinSet(pHandle->H1Port, pHandle->H1Pin));
        }

        switch (pHandle->HallState)
        {
            case STATE_5:
            {
                if (STATE_4 == bPrevHallState)
                {
                    pHandle->Direction       = POSITIVE;
                    pHandle->MeasuredElAngle = pHandle->PhaseShift;
                }
                else if (STATE_1 == bPrevHallState)
                {
                    pHandle->Direction = NEGATIVE;
                    pHandle->MeasuredElAngle =
                        (int16_t)(pHandle->PhaseShift + S16_60_PHASE_SHIFT);
                }
                else
                {
                    /* Nothing to do */
                }
                break;
            }

            case STATE_1:
            {
                if (STATE_5 == bPrevHallState)
                {
                    pHandle->Direction       = POSITIVE;
                    pHandle->MeasuredElAngle = pHandle->PhaseShift + S16_60_PHASE_SHIFT;
                }
                else if (STATE_3 == bPrevHallState)
                {
                    pHandle->Direction = NEGATIVE;
                    pHandle->MeasuredElAngle =
                        (int16_t)(pHandle->PhaseShift + S16_120_PHASE_SHIFT);
                }
                else
                {
                    /* Nothing to do */
                }
                break;
            }

            case STATE_3:
            {
                if (STATE_1 == bPrevHallState)
                {
                    pHandle->Direction = POSITIVE;
                    pHandle->MeasuredElAngle =
                        (int16_t)(pHandle->PhaseShift + S16_120_PHASE_SHIFT);
                }
                else if (STATE_2 == bPrevHallState)
                {
                    pHandle->Direction = NEGATIVE;
                    pHandle->MeasuredElAngle =
                        (int16_t)(pHandle->PhaseShift + S16_120_PHASE_SHIFT +
                                  S16_60_PHASE_SHIFT);
                }
                else
                {
                    /* Nothing to do */
                }
                break;
            }

            case STATE_2:
            {
                if (STATE_3 == bPrevHallState)
                {
                    pHandle->Direction = POSITIVE;
                    pHandle->MeasuredElAngle =
                        (int16_t)(pHandle->PhaseShift + S16_120_PHASE_SHIFT +
                                  S16_60_PHASE_SHIFT);
                }
                else if (STATE_6 == bPrevHallState)
                {
                    pHandle->Direction = NEGATIVE;
                    pHandle->MeasuredElAngle =
                        (int16_t)(pHandle->PhaseShift - S16_120_PHASE_SHIFT);
                }
                else
                {
                    /* Nothing to do */
                }
                break;
            }

            case STATE_6:
            {
                if (STATE_2 == bPrevHallState)
                {
                    pHandle->Direction = POSITIVE;
                    pHandle->MeasuredElAngle =
                        (int16_t)(pHandle->PhaseShift - S16_120_PHASE_SHIFT);
                }
                else if (STATE_4 == bPrevHallState)
                {
                    pHandle->Direction = NEGATIVE;
                    pHandle->MeasuredElAngle =
                        (int16_t)(pHandle->PhaseShift - S16_60_PHASE_SHIFT);
                }
                else
                {
                    /* Nothing to do */
                }
                break;
            }

            case STATE_4:
            {
                if (STATE_6 == bPrevHallState)
                {
                    pHandle->Direction = POSITIVE;
                    pHandle->MeasuredElAngle =
                        (int16_t)(pHandle->PhaseShift - S16_60_PHASE_SHIFT);
                }
                else if (STATE_5 == bPrevHallState)
                {
                    pHandle->Direction       = NEGATIVE;
                    pHandle->MeasuredElAngle = (int16_t)(pHandle->PhaseShift);
                }
                else
                {
                    /* Nothing to do */
                }
                break;
            }

            default:
            {
                /* Bad hall sensor configutarion so update the speed reliability */
                pHandle->SensorIsReliable = false;
                break;
            }
        }

        /* We need to check that the direction has not changed.
           If it is the case, the sign of the current speed can be the opposite of the
           average speed, and the average time can be close to 0 which lead to a
           computed speed close to the infinite, and bring instability. */
        if (pHandle->Direction != PrevDirection)
        {
            /* Setting BufferFilled to 0 will prevent to compute the average speed based
             on the SpeedPeriod buffer values */
            pHandle->BufferFilled = 0U;
            pHandle->SpeedFIFOIdx = 0U;
        }

        if (true == pHandle->HallMtpa)
        {
            pHandle->_Super.hElAngle = pHandle->MeasuredElAngle;
        }
        else
        {
            /* Nothing to do */
        }

        /* Reset incremental value */
        pHandle->IncrementElAngle = pHandle->_Super.hElAngle - pHandle->MeasuredElAngle;

        /* Discard first capture */
        if (0U == pHandle->FirstCapt)
        {
            pHandle->FirstCapt++;
            (void)LL_TIM_IC_GetCaptureCH1(TIMx);
        }
        else
        {
            /* used to validate the average speed measurement */
            if (pHandle->BufferFilled < pHandle->SpeedBufferSize)
            {
                pHandle->BufferFilled++;
            }
            else
            {
                /* Nothing to do */
            }

            /* Store the latest speed acquisition */
            hHighSpeedCapture = (uint16_t)LL_TIM_IC_GetCaptureCH1(TIMx);
            wCaptBuf          = (uint32_t)hHighSpeedCapture;
            hPrscBuf          = (uint16_t)LL_TIM_GetPrescaler(TIMx);

            /* Add the numbers of overflow to the counter */
            wCaptBuf += ((uint32_t)pHandle->OVFCounter) * 0x10000UL;

            if (pHandle->OVFCounter != 0U)
            {
                /* Adjust the capture using prescaler */
                uint16_t hAux;
                hAux = hPrscBuf + 1U;
                wCaptBuf *= hAux;

                if (pHandle->RatioInc)
                {
                    pHandle->RatioInc = false; /* Previous capture caused overflow */
                    /* Don't change prescaler (delay due to preload/update mechanism) */
                }
                else
                {
                    if (LL_TIM_GetPrescaler(TIMx) <
                        pHandle->HALLMaxRatio) /* Avoid OVF w/ very low freq */
                    {
                        LL_TIM_SetPrescaler(
                            TIMx, LL_TIM_GetPrescaler(TIMx) +
                                      1U); /* To avoid OVF during speed decrease */
                        pHandle->RatioInc =
                            true; /* new prsc value updated at next capture only */
                    }
                    else
                    {
                        /* Nothing to do */
                    }
                }
            }
            else
            {
                /* If prsc preload reduced in last capture, store current register + 1 */
                if (pHandle->RatioDec) /* and don't decrease it again */
                {
                    /* Adjust the capture using prescaler */
                    uint16_t hAux;
                    hAux = hPrscBuf + 2U;
                    wCaptBuf *= hAux;

                    pHandle->RatioDec = false;
                }
                else /* If prescaler was not modified on previous capture */
                {
                    /* Adjust the capture using prescaler */
                    uint16_t hAux = hPrscBuf + 1U;
                    wCaptBuf *= hAux;

                    if (hHighSpeedCapture <
                        LOW_RES_THRESHOLD) /* If capture range correct */
                    {
                        if (LL_TIM_GetPrescaler(TIMx) >
                            0U) /* or prescaler cannot be further reduced */
                        {
                            LL_TIM_SetPrescaler(
                                TIMx, LL_TIM_GetPrescaler(TIMx) -
                                          1U); /* Increase accuracy by decreasing prsc */
                            /* Avoid decrementing again in next capt.(register preload
                             * delay) */
                            pHandle->RatioDec = true;
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
            }

            /* Filtering to fast speed... could be a glitch  ? */
            /* the HALL_MAX_PSEUDO_SPEED is temporary in the buffer, and never included in
             * average computation*/
            if (wCaptBuf < pHandle->MinPeriod)
            {
                /* Nothing to do */
#ifdef NOT_IMPLEMENTED /* Not yet implemented */
                pHandle->AvrElSpeedDpp = HALL_MAX_PSEUDO_SPEED;
#endif
            }
            else
            {
                pHandle->ElPeriodSum -=
                    pHandle
                        ->SensorPeriod[pHandle->SpeedFIFOIdx]; /* value we gonna removed
                                                                  from the accumulator */
                if (wCaptBuf >= pHandle->MaxPeriod)
                {
                    pHandle->SensorPeriod[pHandle->SpeedFIFOIdx] =
                        (int32_t)pHandle->MaxPeriod * pHandle->Direction;
                }
                else
                {
                    pHandle->SensorPeriod[pHandle->SpeedFIFOIdx] = (int32_t)wCaptBuf;
                    pHandle->SensorPeriod[pHandle->SpeedFIFOIdx] *= pHandle->Direction;
                }
                pHandle->ElPeriodSum += pHandle->SensorPeriod[pHandle->SpeedFIFOIdx];
                /* Update pointers to speed buffer */
                pHandle->SpeedFIFOIdx++;
                if (pHandle->SpeedFIFOIdx == pHandle->SpeedBufferSize)
                {
                    pHandle->SpeedFIFOIdx = 0U;
                }
                if (pHandle->SensorIsReliable)
                {
                    if ((pHandle->BufferFilled < pHandle->SpeedBufferSize) &&
                        (wCaptBuf != 0U))
                    {
                        uint32_t tempReg = (pHandle->PseudoFreqConv / wCaptBuf) *
                                           (uint32_t)pHandle->Direction;
                        pHandle->AvrElSpeedDpp = (int16_t)tempReg;
                    }
                    else
                    {
                        /* Average speed allow to smooth the mechanical sensors
                         * misalignement */
                        pHandle->AvrElSpeedDpp =
                            (int16_t)((int32_t)pHandle->PseudoFreqConv /
                                      (pHandle->ElPeriodSum /
                                       (int32_t)
                                           pHandle->SpeedBufferSize)); /* Average value */
                    }
                }
                else /* Sensor is not reliable */
                {
                    pHandle->AvrElSpeedDpp = 0;
                }
            }
            /* Reset the number of overflow occurred */
            pHandle->OVFCounter = 0U;
        }
    }
    return (MC_NULL);
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  Example of private method of the class HALL to implement an MC IRQ function
 *         to be called when TIMx update event occurs
 * @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
 */
__weak void *HALL_TIMx_UP_IRQHandler(void *pHandleVoid)
{
    HALL_Handle_t *pHandle = (HALL_Handle_t *)pHandleVoid;  // cstat !MISRAC2012-Rule-11.5
    TIM_TypeDef *TIMx      = pHandle->TIMx;

    if (pHandle->SensorIsReliable)
    {
        uint16_t hMaxTimerOverflow;
        /* an update event occured for this interrupt request generation */
        pHandle->OVFCounter++;

        hMaxTimerOverflow =
            (uint16_t)(((uint32_t)pHandle->HallTimeout * pHandle->OvfFreq) /
                       ((LL_TIM_GetPrescaler(TIMx) + 1U) * 1000U));
        if (pHandle->OVFCounter >= hMaxTimerOverflow)
        {
            /* Set rotor speed to zero */
            pHandle->_Super.hElSpeedDpp = 0;

            /* Reset the electrical angle according the hall sensor configuration */
            HALL_Init_Electrical_Angle(pHandle);

            /* Reset the overflow counter */
            pHandle->OVFCounter = 0U;

            /* Reset first capture flag */
            pHandle->FirstCapt = 0U;

            /* Reset the SensorSpeed buffer*/
            uint8_t bIndex;
            for (bIndex = 0U; bIndex < pHandle->SpeedBufferSize; bIndex++)
            {
                pHandle->SensorPeriod[bIndex] = (int32_t)pHandle->MaxPeriod;
            }
            pHandle->BufferFilled  = 0U;
            pHandle->AvrElSpeedDpp = 0;
            pHandle->SpeedFIFOIdx  = 0U;
            uint32_t tempReg       = pHandle->MaxPeriod * pHandle->SpeedBufferSize;
            pHandle->ElPeriodSum   = (int32_t)tempReg;
        }
    }
    return (MC_NULL);
}

/**
 * @brief  Read the logic level of the three Hall sensor and individuates in this
 *         way the position of the rotor (+/- 30ï¿½). Electrical angle is then
 *         initialized.
 * @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
 */
static void HALL_Init_Electrical_Angle(HALL_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_HALL_SPD_POS_FDB
    if (NULL == pHandle)
    {
        /* Nothing to do */
    }
    else
    {
#endif
        if (DEGREES_120 == pHandle->SensorPlacement)
        {
            pHandle->HallState =
                (uint8_t)((LL_GPIO_IsInputPinSet(pHandle->H3Port, pHandle->H3Pin) << 2U) |
                          (LL_GPIO_IsInputPinSet(pHandle->H2Port, pHandle->H2Pin) << 1U) |
                          LL_GPIO_IsInputPinSet(pHandle->H1Port, pHandle->H1Pin));
        }
        else
        {
            pHandle->HallState =
                (uint8_t)(((LL_GPIO_IsInputPinSet(pHandle->H2Port, pHandle->H2Pin) ^ 1U)
                           << 2U) |
                          (LL_GPIO_IsInputPinSet(pHandle->H3Port, pHandle->H3Pin) << 1U) |
                          LL_GPIO_IsInputPinSet(pHandle->H1Port, pHandle->H1Pin));
        }

        switch (pHandle->HallState)
        {
            case STATE_5:
            {
                pHandle->_Super.hElAngle =
                    (int16_t)(pHandle->PhaseShift + (S16_60_PHASE_SHIFT / 2));
                break;
            }

            case STATE_1:
            {
                pHandle->_Super.hElAngle =
                    (int16_t)(pHandle->PhaseShift + S16_60_PHASE_SHIFT +
                              (S16_60_PHASE_SHIFT / 2));
                break;
            }

            case STATE_3:
            {
                pHandle->_Super.hElAngle =
                    (int16_t)(pHandle->PhaseShift + S16_120_PHASE_SHIFT +
                              (S16_60_PHASE_SHIFT / 2));
                break;
            }

            case STATE_2:
            {
                pHandle->_Super.hElAngle =
                    (int16_t)(pHandle->PhaseShift - S16_120_PHASE_SHIFT -
                              (S16_60_PHASE_SHIFT / 2));
                break;
            }

            case STATE_6:
            {
                pHandle->_Super.hElAngle =
                    (int16_t)(pHandle->PhaseShift - S16_60_PHASE_SHIFT -
                              (S16_60_PHASE_SHIFT / 2));
                break;
            }

            case STATE_4:
            {
                pHandle->_Super.hElAngle =
                    (int16_t)(pHandle->PhaseShift - (S16_60_PHASE_SHIFT / 2));
                break;
            }

            default:
            {
                /* Bad hall sensor configutarion so update the speed reliability */
                pHandle->SensorIsReliable = false;
                break;
            }
        }
        /* Reset incremental value */
        pHandle->IncrementElAngle = 0;

        /* Initialize the measured angle */
        pHandle->MeasuredElAngle = pHandle->_Super.hElAngle;
        pHandle->CompSpeed       = 0;

#ifdef NULL_PTR_CHECK_HALL_SPD_POS_FDB
    }
#endif
}

#ifdef NOT_IMPLEMENTED /* Not yet implemented */
/**
 * @brief  It could be used to set istantaneous information on rotor mechanical
 *         angle.
 *         Note: Mechanical angle management is not implemented in this
 *         version of Hall sensor class.
 * @param  pHandle pointer on related component instance
 * @param  hMecAngle istantaneous measure of rotor mechanical angle
 */
__weak void HALL_SetMecAngle(HALL_Handle_t *pHandle, int16_t hMecAngle) {}
#endif
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
/** @} */

/************************ (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
