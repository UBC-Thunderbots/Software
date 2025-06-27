/**
  ******************************************************************************
  * @file    sto_pll_speed_pos_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the State Observer + PLL Speed & Position Feedback component of the
  *          Motor Control SDK.
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
  * @ingroup SpeednPosFdbk_STO_PLL
  */

/* Includes ------------------------------------------------------------------*/
#include "sto_pll_speed_pos_fdbk.h"
#include "mc_math.h"



/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk
  * @{
  */

/**
  * @defgroup SpeednPosFdbk_STO_PLL State Observer PLL Speed & Position Feedback
  * @brief State Observer with PLL Speed & Position Feedback component of MCSDK
  *
  * This component uses a State Observer coupled with a software PLL to provide an estimation of
  * the speed and the position of the rotor of the motor.
  *
  * See the [Speed & position feedback sensorless chapter of the User Manual](speed_pos_sensorless_bemf_reconstruction.md) for more details on the sensorless algorithm.
  * @{
  */

/* Private defines -----------------------------------------------------------*/
#define C6_COMP_CONST1  ((int32_t)1043038)
#define C6_COMP_CONST2  ((int32_t)10430)

/* Private function prototypes -----------------------------------------------*/
static void STO_Store_Rotor_Speed(STO_PLL_Handle_t *pHandle, int16_t hRotor_Speed);
static int16_t STO_ExecutePLL(STO_PLL_Handle_t *pHandle, int16_t hBemf_alfa_est, int16_t hBemf_beta_est);
static void STO_InitSpeedBuffer(STO_PLL_Handle_t *pHandle);


/**
  * @brief  Initializes the @p pHandle of STate Observer (STO) PLL component.
  *
  */
__weak void STO_PLL_Init(STO_PLL_Handle_t *pHandle)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    int16_t htempk;
    int32_t wAux;

    pHandle->ConsistencyCounter = pHandle->StartUpConsistThreshold;
    pHandle->EnableDualCheck = true;

    wAux = ((int32_t)1);
    pHandle->F3POW2 = 0U;

    htempk = (int16_t)(C6_COMP_CONST1 / pHandle->hF2);

    while (htempk != 0)
    {
      htempk /= ((int16_t)2);
      wAux *= ((int32_t)2);
      pHandle->F3POW2++;
    }

    pHandle->hF3 = (int16_t)wAux;
    wAux = ((int32_t)(pHandle->hF2)) * pHandle->hF3;
    pHandle->hC6 = (int16_t)(wAux / C6_COMP_CONST2);

    STO_PLL_Clear(pHandle);

    PID_HandleInit(&pHandle->PIRegulator);

    /* Acceleration measurement set to zero */
    pHandle->_Super.hMecAccelUnitP = 0;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
  return;
}

/**
  * @brief  Necessary empty return to implement fictitious IRQ_Handler.
  * 
  * @param  pHandle: Handler of the current instance of the STO component.
  * @param  uint8_t: Fictitious interrupt flag.
  */
//cstat !RED-func-no-effect
__weak void STO_PLL_Return(STO_PLL_Handle_t *pHandle, uint8_t flag)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if ((MC_NULL == (void *)pHandle) || ((uint8_t)0 == flag))
  {
    /* Nothing to do */
  }
  else
  {
    /* Nothing to do */
  }
#endif
  return;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
  * @brief  Calculates the estimated electrical angle.
  * 
  * Executes Luenberger state observer equations and calls
  * PLL to compute a new speed estimation and
  * update the estimated electrical angle.
  * 
  * @param  pHandle: Handler of the current instance of the STO component.
  * @param  pInput: Pointer to the observer inputs structure.
  * @retval int16_t Rotor electrical angle (s16Degrees).
  */
__weak int16_t STO_PLL_CalcElAngle(STO_PLL_Handle_t *pHandle, Observer_Inputs_t *pInputs)
{
  int16_t retValue;

  if ((MC_NULL == pHandle) || (MC_NULL == pInputs))
  {
    retValue = 0;
  }
  else
  {
    int32_t wAux;
    int32_t wDirection;
    int32_t wIalfa_est_Next;
    int32_t wIbeta_est_Next;
    int32_t wBemf_alfa_est_Next;
    int32_t wBemf_beta_est_Next;
    int16_t hAux;
    int16_t hAux_Alfa;
    int16_t hAux_Beta;
    int16_t hIalfa_err;
    int16_t hIbeta_err;
    int16_t hRotor_Speed;
    int16_t hValfa;
    int16_t hVbeta;

    if (pHandle->wBemf_alfa_est > (((int32_t)pHandle->hF2) * INT16_MAX))
    {
      pHandle->wBemf_alfa_est = INT16_MAX * ((int32_t)pHandle->hF2);
    }
    else if (pHandle->wBemf_alfa_est <= (-INT16_MAX * ((int32_t)pHandle->hF2)))
    {
      pHandle->wBemf_alfa_est = -INT16_MAX * ((int32_t)pHandle->hF2);
    }
    else
    {
      /* Nothing to do */
    }
#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
      hAux_Alfa = (int16_t)(pHandle->wBemf_alfa_est >> pHandle->F2LOG);
#else
      hAux_Alfa = (int16_t)(pHandle->wBemf_alfa_est / pHandle->hF2);
#endif

    if (pHandle->wBemf_beta_est > (INT16_MAX * ((int32_t)pHandle->hF2)))
    {
      pHandle->wBemf_beta_est = INT16_MAX * ((int32_t)pHandle->hF2);
    }
    else if (pHandle->wBemf_beta_est <= (-INT16_MAX * ((int32_t)pHandle->hF2)))
    {
      pHandle->wBemf_beta_est = (-INT16_MAX * ((int32_t)pHandle->hF2));
    }
    else
    {
      /* Nothing to do */
    }
#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
    hAux_Beta = (int16_t)(pHandle->wBemf_beta_est >> pHandle->F2LOG);
#else
    hAux_Beta = (int16_t)(pHandle->wBemf_beta_est / pHandle->hF2);
#endif

    if (pHandle->Ialfa_est > (INT16_MAX * ((int32_t)pHandle->hF1)))
    {
      pHandle->Ialfa_est = INT16_MAX * ((int32_t)pHandle->hF1);
    }
    else if (pHandle->Ialfa_est <= (-INT16_MAX * ((int32_t)pHandle->hF1)))
    {
      pHandle->Ialfa_est = -INT16_MAX * ((int32_t)pHandle->hF1);
    }
    else
    {
      /* Nothing to do */
    }

    if (pHandle->Ibeta_est > (INT16_MAX * ((int32_t)pHandle->hF1)))
    {
      pHandle->Ibeta_est = INT16_MAX * ((int32_t)pHandle->hF1);
    }
    else if (pHandle->Ibeta_est <= (-INT16_MAX * ((int32_t)pHandle->hF1)))
    {
      pHandle->Ibeta_est = -INT16_MAX * ((int32_t)pHandle->hF1);
    }
    else
    {
      /* Nothing to do */
    }

#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
    hIalfa_err = (int16_t)(pHandle->Ialfa_est >> pHandle->F1LOG);
#else
    hIalfa_err = (int16_t)(pHandle->Ialfa_est / pHandle->hF1);
#endif

    hIalfa_err = hIalfa_err - pInputs->Ialfa_beta.alpha;

#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
    hIbeta_err = (int16_t)(pHandle->Ibeta_est >> pHandle->F1LOG);
#else
    hIbeta_err = (int16_t)(pHandle->Ibeta_est / pHandle->hF1);
#endif

    hIbeta_err = hIbeta_err - pInputs->Ialfa_beta.beta;

    wAux = ((int32_t)pInputs->Vbus) * pInputs->Valfa_beta.alpha;
#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
    hValfa = (int16_t)(wAux >> 16); //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
#else
    hValfa = (int16_t)(wAux / 65536);
#endif

    wAux = ((int32_t)pInputs->Vbus) * pInputs->Valfa_beta.beta;

#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
    hVbeta = ( int16_t ) ( wAux >> 16 ); //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
#else
    hVbeta = (int16_t)(wAux / 65536);
#endif

    /*alfa axes observer*/
#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
    hAux = (int16_t)(pHandle->Ialfa_est >> pHandle->F1LOG);
#else
    hAux = (int16_t)(pHandle->Ialfa_est / pHandle->hF1);
#endif

    wAux = ((int32_t)pHandle->hC1) * hAux;
    wIalfa_est_Next = pHandle->Ialfa_est - wAux;

    wAux = ((int32_t)pHandle->hC2) * hIalfa_err;
    wIalfa_est_Next += wAux;

    wAux = ((int32_t)pHandle->hC5) * hValfa;
    wIalfa_est_Next += wAux;

    wAux = ((int32_t)pHandle->hC3) * hAux_Alfa;
    wIalfa_est_Next -= wAux;

    wAux = ((int32_t)pHandle->hC4) * hIalfa_err;
    wBemf_alfa_est_Next = pHandle->wBemf_alfa_est + wAux;

#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
    wAux = (int32_t)hAux_Beta >> pHandle->F3POW2; //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
#else
    wAux = ((int32_t)hAux_Beta) / pHandle->hF3;
#endif

    wAux = wAux * pHandle->hC6;
    wAux = pHandle->_Super.hElSpeedDpp * wAux;
    wBemf_alfa_est_Next += wAux;

    /*beta axes observer*/
#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
    hAux = (int16_t)(pHandle->Ibeta_est >> pHandle->F1LOG);
#else
    hAux = (int16_t)(pHandle->Ibeta_est / pHandle->hF1);
#endif

    wAux = ((int32_t)pHandle->hC1) * hAux;
    wIbeta_est_Next = pHandle->Ibeta_est - wAux;

    wAux = ((int32_t)pHandle->hC2) * hIbeta_err;
    wIbeta_est_Next += wAux;

    wAux = ((int32_t)pHandle->hC5) * hVbeta;
    wIbeta_est_Next += wAux;

    wAux = ((int32_t)pHandle->hC3) * hAux_Beta;
    wIbeta_est_Next -= wAux;

    wAux = ((int32_t)pHandle->hC4) * hIbeta_err;
    wBemf_beta_est_Next = pHandle->wBemf_beta_est + wAux;

#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
    wAux = (int32_t)hAux_Alfa >> pHandle->F3POW2; //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
#else
    wAux = ((int32_t)hAux_Alfa) / pHandle->hF3;
#endif

    wAux = wAux * pHandle->hC6;
    wAux = pHandle->_Super.hElSpeedDpp * wAux;
    wBemf_beta_est_Next -= wAux;

    /*Calls the PLL blockset*/
    pHandle->hBemf_alfa_est = hAux_Alfa;
    pHandle->hBemf_beta_est = hAux_Beta;

    if (0 == pHandle->hForcedDirection)
    {
      /* we are in auxiliary mode, then rely on the speed detected */
      if(pHandle->_Super.hElSpeedDpp >= 0)
      {
        wDirection = 1;
      }
      else
      {
        wDirection = -1;
      }
    }
    else
    {
      /* we are in main sensor mode, use a forced direction */
      wDirection = pHandle->hForcedDirection;
    }

    hAux_Alfa = (int16_t)(hAux_Alfa * wDirection);
    hAux_Beta = (int16_t)(hAux_Beta * wDirection);

    hRotor_Speed = STO_ExecutePLL(pHandle, hAux_Alfa, -hAux_Beta);
    pHandle->_Super.InstantaneousElSpeedDpp = hRotor_Speed;

    STO_Store_Rotor_Speed(pHandle, hRotor_Speed);

    pHandle->_Super.hElAngle += hRotor_Speed;

    /*storing previous values of currents and bemfs*/
    pHandle->Ialfa_est = wIalfa_est_Next;
    pHandle->wBemf_alfa_est = wBemf_alfa_est_Next;

    pHandle->Ibeta_est = wIbeta_est_Next;
    pHandle->wBemf_beta_est = wBemf_beta_est_Next;
    retValue = pHandle->_Super.hElAngle;
  }
  return (retValue);
}

/**
  * @brief  Computes and returns the average mechanical speed.
  * 
  * This method must be called - at least - with the same periodicity
  * on which speed control is executed. It computes and returns - through
  * parameter hMecSpeedUnit - the rotor average mechanical speed,
  * expressed in #SPEED_UNIT. Average is computed considering a FIFO depth
  * equal to bSpeedBufferSizeUnit. Moreover it also computes and returns
  * the reliability state of the sensor.
  * 
  * @param  pHandle: Handler of the current instance of the STO component.
  * @param  pMecSpeedUnit: Pointer to int16_t, used to return the rotor average
  *         mechanical speed (expressed in #SPEED_UNIT).
  * @retval True if the sensor information is reliable, false otherwise.
  */

__weak bool STO_PLL_CalcAvrgMecSpeedUnit(STO_PLL_Handle_t *pHandle, int16_t *pMecSpeedUnit)
{
  bool bAux;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if ((MC_NULL == pHandle) || (MC_NULL == pMecSpeedUnit))
  {
    bAux = false;
  }
  else
  {
#endif
    int32_t wAvrSpeed_dpp = (int32_t)0;
    int32_t wError;
    int32_t wAux;
    int32_t wAvrSquareSpeed;
    int32_t wAvrQuadraticError = 0;
    int32_t wObsBemf, wEstBemf;
    int32_t wObsBemfSq = 0;
    int32_t wEstBemfSq = 0;
    int32_t wEstBemfSqLo;
    bool bIs_Speed_Reliable = false;
    bool bIs_Bemf_Consistent = false;
    uint8_t i, bSpeedBufferSizeUnit = pHandle->SpeedBufferSizeUnit;

    for (i = 0U; i < bSpeedBufferSizeUnit; i++)
    {
      wAvrSpeed_dpp += (int32_t)(pHandle->Speed_Buffer[i]);
    }

    if (0U == bSpeedBufferSizeUnit)
    {
      /* Nothing to do */
    }
    else
    {
      wAvrSpeed_dpp = wAvrSpeed_dpp / ((int16_t)bSpeedBufferSizeUnit);
    }

    for (i = 0U; i < bSpeedBufferSizeUnit; i++)
    {
      wError = ((int32_t)pHandle->Speed_Buffer[i]) - wAvrSpeed_dpp;
      wError = (wError * wError);
      wAvrQuadraticError += wError;
    }

    /* It computes the measurement variance */
    wAvrQuadraticError = wAvrQuadraticError / ((int16_t)bSpeedBufferSizeUnit);

    /* The maximum variance acceptable is here calculated as a function of average speed */
    wAvrSquareSpeed = wAvrSpeed_dpp * wAvrSpeed_dpp;
    int64_t lAvrSquareSpeed = (int64_t)(wAvrSquareSpeed) * pHandle->VariancePercentage;      
    wAvrSquareSpeed = lAvrSquareSpeed / (int16_t)128;
    
    if (wAvrQuadraticError < wAvrSquareSpeed)
    {
      bIs_Speed_Reliable = true;
    }

    /* Computation of Mechanical speed Unit */
    wAux = wAvrSpeed_dpp * ((int32_t)pHandle->_Super.hMeasurementFrequency);
    wAux = wAux * ((int32_t)pHandle->_Super.SpeedUnit);
    wAux = wAux / ((int32_t)pHandle->_Super.DPPConvFactor);
    wAux = wAux / ((int16_t)pHandle->_Super.bElToMecRatio);

    *pMecSpeedUnit = (int16_t)wAux;
    pHandle->_Super.hAvrMecSpeedUnit = (int16_t)wAux;

    pHandle->IsSpeedReliable = bIs_Speed_Reliable;

    /* Bemf Consistency Check algorithm */
    if (true == pHandle->EnableDualCheck) /*do algorithm if it's enabled*/
    {
      /* wAux abs value   */
      //cstat !MISRAC2012-Rule-14.3_b !RED-func-no-effect !RED-cmp-never !RED-cond-never
      wAux = ((wAux < 0) ? (-wAux) : (wAux));
      if (wAux < (int32_t)(pHandle->MaxAppPositiveMecSpeedUnit))
      {
        /* Computation of Observed back-emf */
        wObsBemf = (int32_t)pHandle->hBemf_alfa_est;
        wObsBemfSq = wObsBemf * wObsBemf;
        wObsBemf = (int32_t)pHandle->hBemf_beta_est;
        wObsBemfSq += wObsBemf * wObsBemf;

        /* Computation of Estimated back-emf */
        wEstBemf = (wAux * 32767) / ((int16_t)pHandle->_Super.hMaxReliableMecSpeedUnit);
        wEstBemfSq = (wEstBemf * ((int32_t)pHandle->BemfConsistencyGain)) / 64;
        wEstBemfSq *= wEstBemf;

        /* Computation of threshold */
        wEstBemfSqLo = wEstBemfSq - ((wEstBemfSq / 64) * ((int32_t)pHandle->BemfConsistencyCheck));

        /* Check */
        if (wObsBemfSq > wEstBemfSqLo)
        {
          bIs_Bemf_Consistent = true;
        }
      }

      pHandle->IsBemfConsistent = bIs_Bemf_Consistent;
      pHandle->Obs_Bemf_Level = wObsBemfSq;
      pHandle->Est_Bemf_Level = wEstBemfSq;
    }
    else
    {
      bIs_Bemf_Consistent = true;
    }

    /* Decision making */
    if (false == pHandle->IsAlgorithmConverged)
    {
      bAux = SPD_IsMecSpeedReliable (&pHandle->_Super, pMecSpeedUnit);
    }
    else
    {
      if ((false == pHandle->IsSpeedReliable) || (false == bIs_Bemf_Consistent))
      {
        pHandle->ReliabilityCounter++;
        if (pHandle->ReliabilityCounter >= pHandle->Reliability_hysteresys)
        {
          pHandle->ReliabilityCounter = 0U;
          pHandle->_Super.bSpeedErrorNumber = pHandle->_Super.bMaximumSpeedErrorsNumber;
          bAux = false;
        }
        else
        {
          bAux = SPD_IsMecSpeedReliable (&pHandle->_Super, pMecSpeedUnit);
        }
      }
      else
      {
        pHandle->ReliabilityCounter = 0U;
        bAux = SPD_IsMecSpeedReliable (&pHandle->_Super, pMecSpeedUnit);
      }
    }
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
  return (bAux);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  Computes and updates the average electrical speed.
  * 
  * This method must be called - at least - with the same periodicity
  * on which speed control is executed. It computes and update component
  * variable hElSpeedDpp that is estimated average electrical speed
  * expressed in dpp used for instance in observer equations.
  * Average is computed considering a FIFO depth equal to
  * bSpeedBufferSizedpp.
  * 
  * @param  pHandle: Handler of the current instance of the STO component.
  */
__weak void STO_PLL_CalcAvrgElSpeedDpp(STO_PLL_Handle_t *pHandle)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    int16_t hIndexNew = (int16_t)pHandle->Speed_Buffer_Index;
    int16_t hIndexOld;
    int16_t hIndexOldTemp;
    int32_t wSum = pHandle->DppBufferSum;
    int32_t wAvrSpeed_dpp;
    int16_t hSpeedBufferSizedpp = (int16_t)pHandle->SpeedBufferSizeDpp;
    int16_t hSpeedBufferSizeUnit = (int16_t)pHandle->SpeedBufferSizeUnit;
    int16_t hBufferSizeDiff;

    hBufferSizeDiff = hSpeedBufferSizeUnit - hSpeedBufferSizedpp;

    if (0 == hBufferSizeDiff)
    {
      wSum = wSum + pHandle->Speed_Buffer[hIndexNew] - pHandle->SpeedBufferOldestEl;
    }
    else
    {
      hIndexOldTemp = hIndexNew + hBufferSizeDiff;

      if (hIndexOldTemp >= hSpeedBufferSizeUnit)
      {
        hIndexOld = hIndexOldTemp - hSpeedBufferSizeUnit;
      }
      else
      {
        hIndexOld = hIndexOldTemp;
      }

      wSum = wSum + pHandle->Speed_Buffer[hIndexNew] - pHandle->Speed_Buffer[hIndexOld];
    }

#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
  //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
  wAvrSpeed_dpp = wSum >> pHandle->SpeedBufferSizeDppLOG;
#else
  if ((int16_t )0 == hSpeedBufferSizedpp)
  {
    /* Nothing to do */
    wAvrSpeed_dpp = wSum;
  }
  else
  {
    wAvrSpeed_dpp = wSum / hSpeedBufferSizedpp;
  }
#endif
    pHandle->_Super.hElSpeedDpp = (int16_t)wAvrSpeed_dpp;
    pHandle->DppBufferSum = wSum;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  Clears state observer component by re-initializing private variables in @p pHandle.
  * 
  */
__weak void STO_PLL_Clear(STO_PLL_Handle_t *pHandle)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->Ialfa_est = (int32_t)0;
    pHandle->Ibeta_est = (int32_t)0;
    pHandle->wBemf_alfa_est = (int32_t)0;
    pHandle->wBemf_beta_est = (int32_t)0;
    pHandle->_Super.hElAngle = (int16_t)0;
    pHandle->_Super.hElSpeedDpp = (int16_t)0;
    pHandle->ConsistencyCounter = 0u;
    pHandle->ReliabilityCounter = 0u;
    pHandle->IsAlgorithmConverged = false;
    pHandle->IsBemfConsistent = false;
    pHandle->Obs_Bemf_Level = (int32_t)0;
    pHandle->Est_Bemf_Level = (int32_t)0;
    pHandle->DppBufferSum = (int32_t)0;
    pHandle->ForceConvergency = false;
    pHandle->ForceConvergency2 = false;

    STO_InitSpeedBuffer(pHandle);
    PID_SetIntegralTerm(& pHandle->PIRegulator, (int32_t)0);
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  Stores in @p pHandle the latest calculated value of @p hRotor_Speed.
  * 
  */
inline static void STO_Store_Rotor_Speed(STO_PLL_Handle_t *pHandle, int16_t hRotor_Speed)
{
  uint8_t bBuffer_index = pHandle->Speed_Buffer_Index;

  bBuffer_index++;
  if (bBuffer_index == pHandle->SpeedBufferSizeUnit)
  {
    bBuffer_index = 0U;
  }

  pHandle->SpeedBufferOldestEl = pHandle->Speed_Buffer[bBuffer_index];
  pHandle->Speed_Buffer[bBuffer_index] = hRotor_Speed;
  pHandle->Speed_Buffer_Index = bBuffer_index;
}

/**
  * @brief  Executes PLL algorithm for rotor position extraction from B-emf alpha and beta.
  * 
  * @param  pHandle: Handler of the current instance of the STO component.
  * @param  hBemf_alfa_est: Estimated Bemf alpha on the stator reference frame.
  * @param  hBemf_beta_est: Estimated Bemf beta on the stator reference frame.
  * @retval 
  */
inline static int16_t STO_ExecutePLL(STO_PLL_Handle_t *pHandle, int16_t hBemf_alfa_est, int16_t hBemf_beta_est)
{
  int32_t wAlfa_Sin_tmp;
  int32_t wBeta_Cos_tmp;
  Trig_Components Local_Components;
  int16_t hAux1;
  int16_t hAux2;
  int16_t hOutput;

  Local_Components = MCM_Trig_Functions(pHandle->_Super.hElAngle);

  /* Alfa & Beta BEMF multiplied by Cos & Sin*/
  wAlfa_Sin_tmp = ((int32_t )hBemf_alfa_est) * ((int32_t )Local_Components.hSin);
  wBeta_Cos_tmp = ((int32_t )hBemf_beta_est) * ((int32_t )Local_Components.hCos);

#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
  hAux1 = (int16_t)(wBeta_Cos_tmp >> 15); //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
#else
  hAux1 = (int16_t)(wBeta_Cos_tmp / 32768);
#endif

#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
  hAux2 = (int16_t)(wAlfa_Sin_tmp >> 15); //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
#else
  hAux2 = (int16_t)(wAlfa_Sin_tmp / 32768);
#endif

  /* Speed PI regulator */
  hOutput = PI_Controller(& pHandle->PIRegulator, (int32_t)(hAux1 ) - hAux2);
  return (hOutput);
}

/**
  * @brief  Clears the estimated speed buffer in @p pHandle.
  *
  */
static void STO_InitSpeedBuffer(STO_PLL_Handle_t * pHandle)
{
  uint8_t b_i;
  uint8_t bSpeedBufferSize = pHandle->SpeedBufferSizeUnit;

  /*init speed buffer*/
  for (b_i = 0U; b_i < bSpeedBufferSize; b_i++)
  {
    pHandle->Speed_Buffer[b_i] = (int16_t)0;
  }
  pHandle->Speed_Buffer_Index = 0U;
  pHandle->SpeedBufferOldestEl = (int16_t)0;

  return;
}

/**
  * @brief  Checks if the state observer algorithm converged.
  * 
  * Internally performs a set of checks necessary to state whether
  * the state observer algorithm converged. To be periodically called
  * during motor open-loop ramp-up (e.g. at the same frequency of
  * SPD_CalcElAngle), it returns true if the estimated angle and speed
  * can be considered reliable, false otherwise.
  * 
  * @param  pHandle: Handler of the current instance of the STO component.
  * @param  hForcedMecSpeedUnit: Mechanical speed in 0.1Hz unit as forced by VSS.
  * @retval bool True if the estimated angle and speed are reliables, false otherwise.
  */
__weak bool STO_PLL_IsObserverConverged(STO_PLL_Handle_t *pHandle, int16_t *phForcedMecSpeedUnit)
{
  bool bAux = false;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if ((MC_NULL == pHandle) || (MC_NULL == phForcedMecSpeedUnit))
  {
    /* Nothing to do */
  }
  else
  {
#endif
    int16_t hEstimatedSpeedUnit;
    int16_t hUpperThreshold;
    int16_t hLowerThreshold;
    int32_t wAux;
    int32_t wtemp;

    if (true == pHandle->ForceConvergency2)
    {
      *phForcedMecSpeedUnit = pHandle->_Super.hAvrMecSpeedUnit;
    }

    if (true == pHandle->ForceConvergency)
    {
      bAux = true;
      pHandle->IsAlgorithmConverged = true;
      pHandle->_Super.bSpeedErrorNumber = 0U;
    }
    else
    {
      hEstimatedSpeedUnit = pHandle->_Super.hAvrMecSpeedUnit;

      wtemp = ((int32_t)hEstimatedSpeedUnit) * ((int32_t)*phForcedMecSpeedUnit);

      if (wtemp > 0)
      {
        if (hEstimatedSpeedUnit < 0)
        {
          hEstimatedSpeedUnit = -hEstimatedSpeedUnit;
        }

        if (*phForcedMecSpeedUnit < 0)
        {
          *phForcedMecSpeedUnit = -*phForcedMecSpeedUnit;
        }
        wAux = ((int32_t)*phForcedMecSpeedUnit) * ((int16_t)pHandle->SpeedValidationBand_H);
        hUpperThreshold = (int16_t)(wAux / ((int32_t)16));

        wAux = ((int32_t)*phForcedMecSpeedUnit) * ((int16_t)pHandle->SpeedValidationBand_L);
        hLowerThreshold = (int16_t)(wAux / ((int32_t)16));

        /* If the variance of the estimated speed is low enough...*/
        if (true == pHandle->IsSpeedReliable)
        {
          if ((uint16_t)hEstimatedSpeedUnit > pHandle->MinStartUpValidSpeed)
          {
            /*...and the estimated value is quite close to the expected value... */
            if (hEstimatedSpeedUnit >= hLowerThreshold)
            {
              if (hEstimatedSpeedUnit <= hUpperThreshold)
              {
                pHandle->ConsistencyCounter++;

                /*... for hConsistencyThreshold consecutive times... */
                if (pHandle->ConsistencyCounter >= pHandle->StartUpConsistThreshold)
                {

                  /* the algorithm converged.*/
                  bAux = true;
                  pHandle->IsAlgorithmConverged = true;
                  pHandle->_Super.bSpeedErrorNumber = 0U;
                }
              }
              else
              {
                pHandle->ConsistencyCounter = 0U;
              }
            }
            else
            {
              pHandle->ConsistencyCounter = 0U;
            }
          }
          else
          {
            pHandle->ConsistencyCounter = 0U;
          }
        }
        else
        {
          pHandle->ConsistencyCounter = 0U;
        }
      }
    }
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
  return (bAux);
}

/**
  * @brief  Exports estimated Bemf alpha-beta from @p pHandle.
  * 
  * @retval alphabeta_t Bemf alpha-beta.
  */
__weak alphabeta_t STO_PLL_GetEstimatedBemf(STO_PLL_Handle_t *pHandle)
{
  alphabeta_t vaux;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    vaux.alpha = 0;
    vaux.beta = 0;
  }
  else
  {
#endif
    vaux.alpha = pHandle->hBemf_alfa_est;
    vaux.beta = pHandle->hBemf_beta_est;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
  return (vaux);
}


/**
  * @brief  Exports from @p pHandle the stator current alpha-beta as estimated by state observer.
  * 
  * @retval alphabeta_t State observer estimated stator current Ialpha-beta.
  */
__weak alphabeta_t STO_PLL_GetEstimatedCurrent(STO_PLL_Handle_t *pHandle)
{
  alphabeta_t iaux;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    iaux.alpha = 0;
    iaux.beta = 0;
  }
  else
  {
#endif
#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
  //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
  iaux.alpha = (int16_t)(pHandle->Ialfa_est >> pHandle->F1LOG);
#else
  iaux.alpha = (int16_t)(pHandle->Ialfa_est / pHandle->hF1);
#endif

#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
  //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
  iaux.beta = (int16_t)(pHandle->Ibeta_est >> pHandle->F1LOG);
#else
  iaux.beta = (int16_t)(pHandle->Ibeta_est / pHandle->hF1);
#endif
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
  return (iaux);
}

/**
  * @brief  Exports current observer gains from @p pHandle and to parameters @p phC2 and @p phC4.
  * 
  */
__weak void STO_PLL_GetObserverGains(STO_PLL_Handle_t *pHandle, int16_t *phC2, int16_t *phC4)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if ((MC_NULL == pHandle) || (MC_NULL == phC2) || (MC_NULL == phC4))
  {
    /* Nothing to do */
  }
  else
  {
#endif
    *phC2 = pHandle->hC2;
    *phC4 = pHandle->hC4;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  Stores in @p pHandle the new values @p hhC1 and @p hhC2 for observer gains.
  * 
  */
__weak void STO_PLL_SetObserverGains(STO_PLL_Handle_t *pHandle, int16_t hhC1, int16_t hhC2)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->hC2 = hhC1;
    pHandle->hC4 = hhC2;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  Exports current PLL gains from @p pHandle to @p pPgain and @p pIgain.
  * 
  */
__weak void STO_GetPLLGains(STO_PLL_Handle_t *pHandle, int16_t *pPgain, int16_t *pIgain)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if ((MC_NULL == pHandle) || (MC_NULL == pPgain) || (MC_NULL == pIgain))
  {
    /* Nothing to do */
  }
  else
  {
#endif
    *pPgain = PID_GetKP(& pHandle->PIRegulator);
    *pIgain = PID_GetKI(& pHandle->PIRegulator);
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
}


/**
  * @brief  Stores in @p pHandle the new values @p hPgain and @p hIgain for PLL gains.
  * 
  */
__weak void STO_SetPLLGains(STO_PLL_Handle_t *pHandle, int16_t hPgain, int16_t hIgain)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    PID_SetKP(&pHandle->PIRegulator, hPgain);
    PID_SetKI(&pHandle->PIRegulator, hIgain);
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
}


/**
  * @brief  Empty function. Could be declared to set instantaneous information on rotor mechanical angle.
  * 
  * Note: Mechanical angle management is not implemented in this
  * version of State observer sensor class.
  * 
  * @param  pHandle: Handler of the current instance of the STO component.
  * @param  hMecAngle: Instantaneous measure of rotor mechanical angle.
  */
//cstat !RED-func-no-effect
__weak void STO_PLL_SetMecAngle(STO_PLL_Handle_t *pHandle, int16_t hMecAngle)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if ((MC_NULL == (void *)pHandle) || ((int16_t)0 == hMecAngle))
  {
    /* Nothing to do */
  }
  else
  {
    /* nothing to do */
  }
#endif
}

/**
  * @brief  Resets the PLL integral term during on-the-fly startup.
  * 
  * @param  pHandle: Handler of the current instance of the STO component.
  */
__weak void STO_OTF_ResetPLL(STO_Handle_t * pHandle)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    STO_PLL_Handle_t *pHdl = (STO_PLL_Handle_t *)pHandle->_Super; //cstat !MISRAC2012-Rule-11.3
    PID_SetIntegralTerm(&pHdl->PIRegulator, (int32_t)0);
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
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
  * @brief  Resets the PLL integral term.
  * 
  * @param  pHandle: Handler of the current instance of the STO component.
  */
__weak void STO_ResetPLL(STO_PLL_Handle_t *pHandle)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    PID_SetIntegralTerm(&pHandle->PIRegulator, (int32_t)0);
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  Sends locking info for PLL.
  * 
  * @param  pHandle: Handler of the current instance of the STO component.
  * @param  hElSpeedDpp: Estimated average electrical speed expressed in dpp.
  * @param  hElAngle: Estimated electrical angle expressed in s16Degrees.
  */
__weak void STO_SetPLL(STO_PLL_Handle_t *pHandle, int16_t hElSpeedDpp, int16_t hElAngle)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    PID_SetIntegralTerm(&pHandle->PIRegulator, ((int32_t)hElSpeedDpp)
                                              * (int32_t)(PID_GetKIDivisor(&pHandle->PIRegulator)));
    pHandle->_Super.hElAngle = hElAngle;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  Exports estimated Bemf squared level stored in @p pHandle.
  * 
  * @retval int32_t Magnitude of estimated Bemf Level squared based on speed measurement.
  */
__weak int32_t STO_PLL_GetEstimatedBemfLevel(STO_PLL_Handle_t *pHandle)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  return ((MC_NULL == pHandle) ? 0 : pHandle->Est_Bemf_Level);
#else
  return (pHandle->Est_Bemf_Level);
#endif
}

/**
  * @brief  Exports observed Bemf squared level stored in @p pHandle.
  *
  * @retval int32_t Magnitude of observed Bemf level squared.
  */
__weak int32_t STO_PLL_GetObservedBemfLevel(STO_PLL_Handle_t *pHandle)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  return ((MC_NULL == pHandle) ? 0 : pHandle->Obs_Bemf_Level);
#else
  return (pHandle->Obs_Bemf_Level);
#endif
}

/**
  * @brief  Enables/Disables additional reliability check based on observed Bemf.
  * 
  * @param  pHandle: Handler of the current instance of the STO component.
  * @param  bSel Enable/Disable check.
  */
__weak void STO_PLL_BemfConsistencyCheckSwitch(STO_PLL_Handle_t *pHandle, bool bSel)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->EnableDualCheck = bSel;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  Checks if the Bemf is consistent.
  * 
  * @param  pHandle: Handler of the current instance of the STO component.
  * @retval bool True when observed Bemfs are consistent with expectation, false otherwise.
  */
__weak bool STO_PLL_IsBemfConsistent(STO_PLL_Handle_t *pHandle)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  return ((MC_NULL == pHandle) ? false : pHandle->IsBemfConsistent);
#else
  return (pHandle->IsBemfConsistent);
#endif
}

/**
  * @brief  Checks the value of the variance.
  * 
  * @param  pHandle: Handler of the current instance of the STO component.
  * @retval bool True if the speed measurement variance is lower than threshold VariancePercentage, false otherwise.
  */
__weak bool STO_PLL_IsVarianceTight(const STO_Handle_t *pHandle)
{
  bool tempStatus;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    tempStatus = false;
  }
  else
  {
#endif
    STO_PLL_Handle_t *pHdl = (STO_PLL_Handle_t *)pHandle->_Super; //cstat !MISRAC2012-Rule-11.3
    tempStatus = pHdl->IsSpeedReliable;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
  return (tempStatus);
}

/**
  * @brief  Forces the state-observer to declare convergency.
  * 
  * @param  pHandle: Handler of the current instance of the STO component.
  */
__weak void STO_PLL_ForceConvergency1(STO_Handle_t *pHandle)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    STO_PLL_Handle_t *pHdl = (STO_PLL_Handle_t *)pHandle->_Super; //cstat !MISRAC2012-Rule-11.3
    pHdl->ForceConvergency = true;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  Forces the state-observer to declare convergency.
  * 
  * @param  pHandle: Handler of the current instance of the STO component.
  */
__weak void STO_PLL_ForceConvergency2(STO_Handle_t *pHandle)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    STO_PLL_Handle_t *pHdl = (STO_PLL_Handle_t *)pHandle->_Super; //cstat !MISRAC2012-Rule-11.3
    pHdl->ForceConvergency2 = true;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  Sets the Absolute value of minimum mechanical speed (expressed in
  *         the unit defined by #SPEED_UNIT) required to validate the start-up.
  * 
  * @param  pHandle: Handler of the current instance of the STO component.
  * @param  hMinStartUpValidSpeed: Absolute value of minimum mechanical speed.
  */
__weak void STO_SetMinStartUpValidSpeedUnit(STO_PLL_Handle_t *pHandle, uint16_t hMinStartUpValidSpeed)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->MinStartUpValidSpeed = hMinStartUpValidSpeed;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  Sets the rotation @p direction in @p pHandle.
  */
__weak void STO_SetDirection(STO_PLL_Handle_t *pHandle, int8_t direction)
{
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->hForcedDirection = direction;
#ifdef NULL_PTR_STO_PLL_SPD_POS_FDB
  }
#endif
}


/**
  * @}
  */

/**
  * @}
  */

/** @} */

/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
