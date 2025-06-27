/**
  ******************************************************************************
  * @file    flux_weakening_ctrl.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implements the Flux Weakening
  *          Control component of the Motor Control SDK.
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
  * @ingroup FluxWeakeningCtrl
  */

/* Includes ------------------------------------------------------------------*/
#include "flux_weakening_ctrl.h"
#include "mc_math.h"
#include "mc_type.h"
#include "pid_regulator.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup FluxWeakeningCtrl Flux Weakening Control
  * @brief Flux Weakening (FW) Control component of the Motor Control SDK
  *
  * The Flux Weakening Control component modifies Idq reference to reach a speed higher than rated one.
  * To do so it uses its own PID controller to control the current reference and acts also on the PID speed controller.  
  *
  * Flux Weakening Control component needs to be initialized before it can be used. This is done with the FW_Init() 
  * function. Ensure that PID speed has been correctly initialized prior to use flux weakiening component.
  *
  * The controller functions implemented by the FW_CalcCurrRef() functions is based on 16-bit integer arithmetics 
  * The controller output values returned by this functions is also 16-bit integers. This makes it possible to use this 
  * component efficiently on all STM2 MCUs. 
  *
  * For more information, please refer to [Flux Weakening documentation](flux_weakening_control.md)
  *
  * @{
  */

/**
  * @brief  Initializes flux weakening component handler, it should be called 
  *         once during Motor Control initialization.
  * @param  pHandle pointer to flux weakening component handler.
  * @param  pPIDSpeed pointer to speed PID strutcture.
  * @param  PIDFluxWeakeningHandle pointer to FW PID strutcture.
  */
__weak void FW_Init(FW_Handle_t *pHandle, PID_Handle_t *pPIDSpeed, PID_Handle_t *pPIDFluxWeakeningHandle)
{
#ifdef NULL_PTR_CHECK_FLUX_WEAK
  if (NULL == pHandle)
  {
    /* Mothing to do */
  }
  else
  {
#endif
    pHandle->hFW_V_Ref = pHandle->hDefaultFW_V_Ref;
    pHandle->pFluxWeakeningPID = pPIDFluxWeakeningHandle;
    pHandle->pSpeedPID = pPIDSpeed;
#ifdef NULL_PTR_CHECK_FLUX_WEAK
  }
#endif
}

/**
  * @brief  Clears the Flux weakening internal variables except the target
  *         voltage (hFW_V_Ref). It should be called before each motor restart
  * @param  pHandle pointer to flux weakening component handler.
  */
__weak void FW_Clear(FW_Handle_t *pHandle)
{
  qd_t V_null = {(int16_t)0, (int16_t)0};
#ifdef NULL_PTR_CHECK_FLUX_WEAK
  if (NULL == pHandle)
  {
    /* Mothing to do */
  }
  else
  {
#endif
    PID_SetIntegralTerm(pHandle->pFluxWeakeningPID, (int32_t)0);
    pHandle->AvVolt_qd = V_null;
    pHandle->AvVoltAmpl = (int16_t)0;
    pHandle->hIdRefOffset = (int16_t)0;
#ifdef NULL_PTR_CHECK_FLUX_WEAK
  }
#endif
}

/**
  * @brief  Computes Iqdref according to the flux weakening algorithm.
  *         As soon as the speed increases beyond the nominal one, flux weakening
  *         algorithm takes place and handles Idref value. Finally, accordingly
  *         with new Idref, a new Iqref saturation value is also computed and
  *         put into speed PI. this routine should be called during background task.
  * @param  pHandle pointer to flux weakening component handler.
  * @param  Iqdref current reference that will be
  *         modified, if needed, by the flux weakening algorithm.
  * @retval qd_t Computed Iqdref.
  */
__weak qd_t FW_CalcCurrRef(FW_Handle_t *pHandle, qd_t Iqdref)
{
  qd_t IqdrefRet;
#ifdef NULL_PTR_CHECK_FLUX_WEAK
  if (NULL == pHandle)
  {
    IqdrefRet.d = 0;
    IqdrefRet.q = 0;
  }
  else
  {
#endif
    int32_t wIdRef;
    int32_t wIqSatSq;
    int32_t wIqSat;
    int32_t wAux1;
    int16_t Aux2;
    uint32_t wVoltLimit_Ref;
    int16_t hId_fw;

    /* Computation of the Id contribution coming from flux weakening algorithm */
    wVoltLimit_Ref = ((uint32_t)(pHandle->hFW_V_Ref) * pHandle->hMaxModule) / 1000U;
    Aux2 = MCM_Modulus( pHandle->AvVolt_qd.q, pHandle->AvVolt_qd.d );
    pHandle->AvVoltAmpl = Aux2;

    hId_fw = PI_Controller(pHandle->pFluxWeakeningPID, (int32_t)wVoltLimit_Ref - (int32_t)Aux2);

    /* If the Id coming from flux weakening algorithm (Id_fw) is positive, keep
    unchanged Idref, otherwise sum it to last Idref available when Id_fw was
    zero */
    if (hId_fw >= (int16_t)0)
    {
      pHandle->hIdRefOffset = Iqdref.d;
      wIdRef = (int32_t)Iqdref.d;
    }
    else
    {
      wIdRef = (int32_t)pHandle->hIdRefOffset + hId_fw;
    }

    /* Saturate new Idref to prevent the rotor from being demagnetized */
    if (wIdRef < pHandle->hDemagCurrent)
    {
      wIdRef =  pHandle->hDemagCurrent;
    }
    else
    {
      /* Nothing to do */
    }

    IqdrefRet.d = (int16_t)wIdRef;

    /* New saturation for Iqref */
    wIqSatSq =  pHandle->wNominalSqCurr - (wIdRef * wIdRef);
    wIqSat = MCM_Sqrt(wIqSatSq);

    /* Iqref saturation value used for updating integral term limitations of
    speed PI */
    wAux1 = wIqSat * (int32_t)PID_GetKIDivisor(pHandle->pSpeedPID);

    PID_SetLowerIntegralTermLimit(pHandle->pSpeedPID, -wAux1);
    PID_SetUpperIntegralTermLimit(pHandle->pSpeedPID, wAux1);

    /* Iqref saturation value used for updating integral term limitations of
    speed PI */
    if (Iqdref.q > wIqSat)
    {
      IqdrefRet.q = (int16_t)wIqSat;
    }
    else if (Iqdref.q < -wIqSat)
    {
      IqdrefRet.q = -(int16_t)wIqSat;
    }
    else
    {
      IqdrefRet.q = Iqdref.q;
    }
#ifdef NULL_PTR_CHECK_FLUX_WEAK
  }
#endif
  return (IqdrefRet);
}

//cstat #ATH-shift-bounds
#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
  * @brief  Applies a low-pass filter on both  Vqd voltage components. Filter
  *         bandwidth depends on hVqdLowPassFilterBW parameter. It shall
  *         be called during current controller task.
  * @param  pHandle pointer to flux weakening component handler.
  * @param  Vqd Voltage componets to be averaged.
  */
__weak void FW_DataProcess(FW_Handle_t *pHandle, qd_t Vqd)
{
#ifdef NULL_PTR_CHECK_FLUX_WEAK
  if (NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    int32_t wAux;
    int32_t lowPassFilterBW = (int32_t)(pHandle->hVqdLowPassFilterBW) - (int32_t)1 ;

#ifndef FULL_MISRA_C_COMPLIANCY_FLUX_WEAK
    wAux = (int32_t)(pHandle->AvVolt_qd.q) * lowPassFilterBW;
    wAux += Vqd.q;
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
    pHandle->AvVolt_qd.q = (int16_t)(wAux >> pHandle->hVqdLowPassFilterBWLOG);

    wAux = (int32_t)(pHandle->AvVolt_qd.d) * lowPassFilterBW;
    wAux += Vqd.d;
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
    pHandle->AvVolt_qd.d = (int16_t)(wAux >> pHandle->hVqdLowPassFilterBWLOG);
#else
    wAux = (int32_t)(pHandle->AvVolt_qd.q) * lowPassFilterBW;
    wAux += Vqd.q;

    pHandle->AvVolt_qd.q = (int16_t)(wAux / (int32_t)(pHandle->hVqdLowPassFilterBW));

    wAux = (int32_t)(pHandle->AvVolt_qd.d) * lowPassFilterBW;
    wAux += Vqd.d;

    pHandle->AvVolt_qd.d = (int16_t)(wAux / (int32_t)pHandle->hVqdLowPassFilterBW);
#endif
#ifdef NULL_PTR_CHECK_FLUX_WEAK
  }
#endif
  return;
}


/**
  * @brief  Sets a new value for the voltage reference used by
  *         flux weakening algorithm.
  * @param  pHandle pointer to flux weakening component handler.
  * @param  hNewVref New target voltage value, expressed in tenth of percentage
  *         points of available voltage.
  */
__weak void FW_SetVref(FW_Handle_t *pHandle, uint16_t hNewVref)
{
#ifdef NULL_PTR_CHECK_FLUX_WEAK
  if (NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->hFW_V_Ref = hNewVref;
#ifdef NULL_PTR_CHECK_FLUX_WEAK
  }
#endif
}

/**
  * @brief  Returns the present value of target voltage used by flux
  *         weakening algorihtm.
  * @param  pHandle pointer to flux weakening component handler.
  * @retval int16_t Present target voltage value expressed in tenth of
  *         percentage points of available voltage.
  */
__weak uint16_t FW_GetVref(FW_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_FLUX_WEAK
  return ((NULL == pHandle) ? 0U : pHandle->hFW_V_Ref);
#else
  return (pHandle->hFW_V_Ref);
#endif
}

/**
  * @brief  Returns the present value of voltage actually used by flux
  *         weakening algorihtm.
  * @param  pHandle pointer to flux weakening component handler.
  * @retval int16_t Present averaged phase stator voltage value, expressed
  *         in s16V (0-to-peak), where
  *         PhaseVoltage(V) = [PhaseVoltage(s16A) * Vbus(V)] /[sqrt(3) *32767].
  */
__weak int16_t FW_GetAvVAmplitude(FW_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_FLUX_WEAK
  return ((NULL == pHandle) ? 0 : pHandle->AvVoltAmpl);
#else
  return (pHandle->AvVoltAmpl);
#endif
}

/**
  * @brief  Returns the present voltage actually used by flux
  *         weakening algorihtm as percentage of available voltage.
  * @param  pHandle pointer to flux weakening component handler.
  * @retval uint16_t Present averaged phase stator voltage value, expressed in
  *         tenth of percentage points of available voltage.
  */
__weak uint16_t FW_GetAvVPercentage(FW_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_FLUX_WEAK
  return ((NULL == pHandle) ? 0U
          : (uint16_t)((uint32_t)(pHandle->AvVoltAmpl) * 1000U / (uint32_t)(pHandle->hMaxModule)));
#else
  return ((uint16_t)((uint32_t)(pHandle->AvVoltAmpl) * 1000U / (uint32_t)(pHandle->hMaxModule)));
#endif
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
