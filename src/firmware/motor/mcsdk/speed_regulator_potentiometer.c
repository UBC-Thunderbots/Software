/**
  ******************************************************************************
  * @file    speed_regulator_potentiometer.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the Speed Regulator Potentiometer component of the Motor Control SDK.
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
  *
  * @ingroup SpeedRegulatorPotentiometer
  */

/* Includes ------------------------------------------------------------------*/
#include "speed_regulator_potentiometer.h"

/** @addtogroup MCSDK
  * @{
  */

/**
 * @defgroup SpeedRegulatorPotentiometer Speed Potentiometer
 * @brief Potentiometer for motor speed reference setting compotent of the Motor Control SDK
 * 
 * Reads a potentiometer and sets the mechanical speed reference of the rotor accordingly
 * 
 * The Speed Regulator Potentiometer component reads a potentiometer thro
 * 
 * @todo Complete the documentation of the Speed Regulator Potentiometer component
 * 
 * @{
 */

/* Functions ---------------------------------------------------- */

/**
 * @brief Initializes a Speed Regulator Potentiometer component
 *
 * @param pHandle The Speed Regulator Potentiometer component to initialize
 *
 */
__weak void SRP_Init(SRP_Handle_t *pHandle, SpeednTorqCtrl_Handle_t *pSTC)
{

  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    /* Need to be register with RegularConvManager */
    pHandle->convHandle = RCM_RegisterRegConv(&pHandle->SpeedRegConv);
    pHandle->pSTC = pSTC;
    SRP_Clear(pHandle);
  }
}

/**
 * @brief Resets the internal state of a Speed Regulator Potentiometer component
 *
 * @param pHandle Handle of the Speed Regulator Potentiometer component 
 */
__weak void SRP_Clear(SRP_Handle_t *pHandle )
{
  if (MC_NULL == pHandle)
  {
    /* nothing to do */
  }
  else
  {
    uint16_t aux;
    uint16_t index;
    aux = ( pHandle->MaximumSpeedRange + pHandle->MinimumSpeedRange ) / 2u;
    for ( index = 0u; index < pHandle->LowPassFilterBW; index++ )
    {
      pHandle->aBuffer[index] = aux;
    }
    pHandle->LatestConv = aux;
    pHandle->AvSpeedReg_d = aux;
    pHandle->index = 0;
  }
}

/**
 * @brief 
 * 
 * @param pHandle 
 * @return uint16_t 
 */
static uint16_t SRP_ConvertSpeedRegFiltered( SRP_Handle_t *pHandle )
{
  uint16_t hAux;
  uint8_t vindex;
  uint16_t max = 0, min = 0;
  uint32_t tot = 0u;

  for ( vindex = 0; vindex < pHandle->LowPassFilterBW; )
  {
    hAux = RCM_ExecRegularConv(pHandle->convHandle);

    if ( hAux != 0xFFFFu )
    {
      if ( vindex == 0 )
      {
        min = hAux;
        max = hAux;
      }
      else
      {
        if ( hAux < min )
        {
          min = hAux;
        }
        if ( hAux > max )
        {
          max = hAux;
        }
      }
      vindex++;

      tot += hAux;
    }
  }

  tot -= max;
  tot -= min;
  return ( uint16_t )( tot / ( pHandle->LowPassFilterBW - 2u ) );
}

/**
 * @brief 
 * 
 * @param pHandle 
 * @param DigitValue 
 * @return uint16_t 
 */
static inlineuint16_t SRP_SpeedDigitToSpeedUnit( SRP_Handle_t *pHandle, uint16_t DigitValue )
{
  uint16_t hAux;
  hAux = DigitValue / pHandle->ConversionFactor + pHandle->MinimumSpeedRange;
  return hAux;
}

/**
 * @brief 
 * 
 * @param pHandle 
 * @param SpeedUnitValue 
 * @return uint16_t 
 */
static uint16_t SRP_SpeedUnitToSpeedDigit( SRP_Handle_t *pHandle, int16_t SpeedUnitValue )
{
  uint16_t hAux = 0;

  if ( SpeedUnitValue < 0 ) SpeedUnitValue = -SpeedUnitValue;

  if ( SpeedUnitValue > pHandle->MinimumSpeedRange )
  {
    hAux = (SpeedUnitValue - pHandle->MinimumSpeedRange) * pHandle->ConversionFactor;
  }

  return hAux;
}

/**
  * @brief Reads the potentiometer of an SRP component and filters it to compute an average speed reference
  * 
  * It actually performes the Speed regulator ADC conversion and updates average
  *         value
  * @param  pHandle related SRP_Handle_t
  * @retval bool OutOfSynch signal
  */
__weak bool SRP_CalcAvSpeedRegFilt( SRP_Handle_t *pHandle )
{
  uint32_t wtemp;
  uint16_t hAux;
  uint8_t i;

  hAux = SRP_ConvertSpeedRegFiltered( pHandle );

  if ( hAux != 0xFFFF )
  {
    pHandle->aBuffer[pHandle->index] = hAux;
    wtemp = 0;
    for ( i = 0; i < pHandle->LowPassFilterBW; i++ )
    {
      wtemp += pHandle->aBuffer[i];
    }
    wtemp /= pHandle->LowPassFilterBW;
    pHandle->AvSpeedReg_d = ( uint16_t )wtemp;
    pHandle->LatestConv = hAux;

    if ( pHandle->index < pHandle->LowPassFilterBW - 1 )
    {
      pHandle->index++;
    }
    else
    {
      pHandle->index = 0;
    }
  }
  pHandle->OutOfSynchro = SRP_CheckSpeedRegSync( pHandle );

  return ( pHandle->OutOfSynchro );
}

/**
  * @brief  It actually performes the speed regulator ADC conversion and updates average
  *         value
  * @param  pHandle related SRP_Handle_t
  * @retval bool OutOfSynch signal
  */
// __weak bool SRP_CalcAvSpeedReg( SRP_Handle_t * pHandle )
// {
//   uint32_t wtemp;
//   uint16_t hAux;
//   uint8_t i;

//   hAux = RCM_ExecRegularConv(pHandle->convHandle);

//   if ( hAux != 0xFFFF )
//   {
//     pHandle->aBuffer[pHandle->index] = hAux;
//     wtemp = 0;
//     for ( i = 0; i < pHandle->LowPassFilterBW; i++ )
//     {
//       wtemp += pHandle->aBuffer[i];
//     }
//     wtemp /= pHandle->LowPassFilterBW;
//     pHandle->AvSpeedReg_d = ( uint16_t )wtemp;
//     pHandle->LatestConv = hAux;

//     if ( pHandle->index < pHandle->LowPassFilterBW - 1 )
//     {
//       pHandle->index++;
//     }
//     else
//     {
//       pHandle->index = 0;
//     }
//   }
//   pHandle->OutOfSynchro = SRP_CheckSpeedRegSync( pHandle );

//   return ( pHandle->OutOfSynchro );
// }
__weak bool SRP_CalcAvgSpeedReg()
{
  uint32_t wtemp;
  uint16_t hAux;
  uint8_t i;

  /* Check regular conversion state */
  if (RCM_GetUserConvState() == RCM_USERCONV_IDLE)
  {
    /* if Idle, then program a new conversion request */
    RCM_RequestUserConv(handle);
  }
  else if (RCM_GetUserConvState() == RCM_USERCONV_EOC)
  {
    /* if completed, then read the captured value */
    hAux = RCM_GetUserConv();


    if ( hAux != 0xFFFF )
    {
      pHandle->aBuffer[pHandle->index] = hAux;
      wtemp = 0;
      for ( i = 0; i < pHandle->LowPassFilterBW; i++ )
      {
        wtemp += pHandle->aBuffer[i];
      }
      wtemp /= pHandle->LowPassFilterBW;
      pHandle->AvSpeedReg_d = ( uint16_t )wtemp;
      pHandle->LatestConv = hAux;

      if ( pHandle->index < pHandle->LowPassFilterBW - 1 )
      {
        pHandle->index++;
      }
      else
      {
        pHandle->index = 0;
      }
    }
  }

  pHandle->OutOfSynchro = SRP_CheckSpeedRegSync( pHandle );

  return ( pHandle->OutOfSynchro );
}


/**
  * @brief  It returns OutOfSync check between current potentiometer setting and measured speed
  * @param  pHandle related SRP_Handle_t
  * @retval bool OutOfSynch signal
  */
__weak bool SRP_CheckSpeedRegSync( SRP_Handle_t * pHandle )
{
  bool hAux = false;
  uint16_t speedRegValue = SRP_SpeedDigitToSpeedUnit(pHandle, pHandle->AvSpeedReg_d );
  uint16_t speedRegTol = SRP_SpeedDigitToSpeedUnit(pHandle, pHandle->SpeedAdjustmentRange);
  SpeednPosFdbk_Handle_t *speedHandle;
  speedHandle = STC_GetSpeedSensor(pHandle->pSTC);
  if ((speedHandle->hAvrMecSpeedUnit > (speedRegValue +  speedRegTol)) || (speedHandle->hAvrMecSpeedUnit < (speedRegValue -  speedRegTol)))
  {
    hAux = true;
  }
  return hAux;
}

/**
  * @brief  Executes speed ramp and applies external speed regulator new setpoint if ajustment range is violated
  * @param  pHandle related SRP_Handle_t
  * @retval bool final speed is equal to measured speed
  */
__weak bool SRP_ExecPotRamp( SRP_Handle_t * pHandle )
{
  bool hAux = SRP_CheckSpeedRegSync(pHandle);
  uint16_t PotentiometerReqSpeed = pHandle->LatestConv;
  uint16_t PotentiometerCurrentSpeed = pHandle->AvSpeedReg_d;

  SpeednPosFdbk_Handle_t *speedHandle;
  speedHandle = STC_GetSpeedSensor(pHandle->pSTC);
  int16_t CurrentSpeed = speedHandle->hAvrMecSpeedUnit;
      
  if ((PotentiometerReqSpeed <= PotentiometerCurrentSpeed - pHandle->SpeedAdjustmentRange) ||   // Requested speed must be different from previous one
      (PotentiometerReqSpeed >= PotentiometerCurrentSpeed + pHandle->SpeedAdjustmentRange))
  {
    int16_t deltaSpeed;
    uint16_t RequestedSpeed = SRP_SpeedDigitToSpeedUnit(pHandle, PotentiometerReqSpeed );
    uint32_t hDuration;
    deltaSpeed = (int16_t) RequestedSpeed - CurrentSpeed;
    if (deltaSpeed > 0) hDuration = ((uint32_t) deltaSpeed) * 1000 / pHandle->RampSlope;
    else hDuration = ((uint32_t) (- deltaSpeed)) * 1000 / pHandle->RampSlope;
    if (CurrentSpeed < 0)
    {
      STC_ExecRamp(pHandle->pSTC, (int16_t) (- RequestedSpeed), hDuration);
    }
    else
    {
      STC_ExecRamp(pHandle->pSTC, (int16_t) RequestedSpeed, hDuration);
    }
  }
  return hAux;
}

/**
 * @brief Returns the latest value read from the potentiometer of a Speed Regulator Potentiometer component
 * 
 * @param pHandle Handle of the Speed Regulator Potentiometer component
 */
__weak uint16_t SRP_GetSpeedReg_d( SRP_Handle_t * pHandle )
{
  return ( pHandle->LatestConv );
}

/**
 * @brief Returns the speed reference computed by a Speed Regulator Potentiometer 
 *        component expressed in u16digits.
 * 
 * This speed reference is an average computed over the last values read from the potentiometer.
 * 
 * The number of values on which the average is computed is given by SRP_Handle_t::LowPassFilterBW.
 * 
 * @param pHandle Handle of the Speed Regulator Potentiometer component
 */
__weak uint16_t SRP_GetAvSpeedReg_d( SRP_Handle_t * pHandle )
{
  return ( pHandle->AvSpeedReg_d );
}

/**
 * @brief Returns the speed reference computed by a Speed Regulator Potentiometer component 
 *         expressed in #SPEED_UNIT
 * 
 * @param pHandle Handle of the Speed Regulator Potentiometer component
 */
__weak uint16_t SRP_GetAvSpeedReg_SU( SRP_Handle_t * pHandle )
{
  uint16_t temp = SRP_SpeedDigitToSpeedUnit(pHandle, pHandle->AvSpeedReg_d );
  return ( temp );
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
