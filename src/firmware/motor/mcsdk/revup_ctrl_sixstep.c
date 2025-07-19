/**
  ******************************************************************************
  * @file    revup_ctrl_sixstep.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides the functions that implement the features
  *          of the Rev-Up Control component for Six-Step drives of the Motor 
  *          Control SDK.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  * @ingroup RevUpCtrl6S
  */

/* Includes ------------------------------------------------------------------*/
#include "revup_ctrl_sixstep.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup RevUpCtrl
  *
  * @{
  */

/** @defgroup RevUpCtrl6S Six-Step Rev-Up Control component
  * @brief Rev-Up Control component used to start motor driven with the Six-Step technique
  *
  * @{
  */


/* Private defines -----------------------------------------------------------*/

/**
  * @brief Timeout used to reset integral term of PLL.
  *  It is expressed in ms.
  *
  */
#define RUC_OTF_PLL_RESET_TIMEOUT 100u

/* Private functions ----------------------------------------------------------*/

/* Returns the  mechanical speed of a selected phase */
static int16_t RUC_GetPhaseFinalMecSpeed01Hz(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase)
{
  int16_t hRetVal = 0;

  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    hRetVal = pHandle->ParamsData[bPhase].hFinalMecSpeedUnit;
  }
  return (hRetVal);
}

/**
  * @brief  Initialize and configure the 6-Step RevUpCtrl Component
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  * @param  pSTC: Pointer on speed and torque controller structure.
  * @param  pVSS: Pointer on virtual speed sensor structure.
  */
__weak void RUC_Init(	RevUpCtrl_Handle_t *pHandle,
						SpeednTorqCtrl_Handle_t *pSTC,
						VirtualSpeedSensor_Handle_t *pVSS)
{
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    RevUpCtrl_PhaseParams_t *pRUCPhaseParams = &pHandle->ParamsData[0];
    uint8_t bPhase = 0U;

    pHandle->pSTC = pSTC;
    pHandle->pVSS = pVSS;
    pHandle->EnteredZone1 = false;

    while ((pRUCPhaseParams != MC_NULL) && (bPhase < RUC_MAX_PHASE_NUMBER))
    {
      /* Dump HF data for now HF data are forced to 16 bits*/
      pRUCPhaseParams = (RevUpCtrl_PhaseParams_t * )pRUCPhaseParams->pNext; //cstat !MISRAC2012-Rule-11.5
      bPhase++;
    }

    if (0U == bPhase)
    {
      /* nothing to do error */
    }
    else
    {
      pHandle->ParamsData[bPhase - 1u].pNext = MC_NULL;

      pHandle->bPhaseNbr = bPhase;

    }
  }
}

/*
  * @brief  Initialize internal 6-Step RevUp controller state.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  * @param  hMotorDirection: Rotor rotation direction.
  *         This parameter must be -1 or +1.
  */

__weak void RUC_Clear(RevUpCtrl_Handle_t *pHandle, int16_t hMotorDirection)
{
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    VirtualSpeedSensor_Handle_t *pVSS = pHandle->pVSS;
    SpeednTorqCtrl_Handle_t *pSTC = pHandle->pSTC;
    RevUpCtrl_PhaseParams_t *pPhaseParams = pHandle->ParamsData;

    pHandle->hDirection = hMotorDirection;
    pHandle->EnteredZone1 = false;

    /*Initializes the rev up stages counter.*/
    pHandle->bStageCnt = 0U;

    /* Calls the clear method of VSS.*/
    VSS_Clear(pVSS);

    /* Sets the STC in torque mode.*/
    STC_SetControlMode(pSTC, MCM_TORQUE_MODE);

    /* Sets the mechanical starting angle of VSS.*/
    VSS_SetMecAngle(pVSS, pHandle->hStartingMecAngle * hMotorDirection);

    /* Sets to zero the starting torque of STC */
    (void)STC_ExecRamp(pSTC, STC_GetDutyCycleRef(pSTC), 0U);

    /* Gives the first command to STC and VSS.*/
    (void)STC_ExecRamp(pSTC, pPhaseParams->hFinalPulse, (uint32_t)(pPhaseParams->hDurationms));

    VSS_SetMecAcceleration(pVSS, pPhaseParams->hFinalMecSpeedUnit * hMotorDirection, pPhaseParams->hDurationms );

    /* Compute hPhaseRemainingTicks.*/
    pHandle->hPhaseRemainingTicks = (uint16_t)((((uint32_t)pPhaseParams->hDurationms)
                                              * ((uint32_t)pHandle->hRUCFrequencyHz))
                                              / 1000U );

    pHandle->hPhaseRemainingTicks++;

    /*Set the next phases parameter pointer.*/
    pHandle->pCurrentPhaseParams = (RevUpCtrl_PhaseParams_t * )pPhaseParams->pNext; //cstat !MISRAC2012-Rule-11.5

  }
}

/**
  * @brief  Update rev-up duty cycle relative to actual Vbus value to be applied
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  * @param  BusVHandle: pointer to the bus voltage sensor
  */
__weak void RUC_UpdatePulse(RevUpCtrl_Handle_t *pHandle, BusVoltageSensor_Handle_t *BusVHandle)
{
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    uint16_t tPulseUpdateFactor = 10 * NOMINAL_BUS_VOLTAGE_V / VBS_GetAvBusVoltage_V(BusVHandle);
    pHandle->PulseUpdateFactor = tPulseUpdateFactor;						
  }
}

/*
  * @brief  6-Step Main revup controller procedure executing overall programmed phases.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  *  @retval Boolean set to true when entire revup phases have been completed.
  */
__weak bool RUC_Exec(RevUpCtrl_Handle_t *pHandle)
{
  bool retVal = true;

  if (MC_NULL == pHandle)
  {
    retVal = false;
  }
  else
  {
    if (pHandle->hPhaseRemainingTicks > 0U)
    {
      /* Decrease the hPhaseRemainingTicks.*/
      pHandle->hPhaseRemainingTicks--;

    } /* hPhaseRemainingTicks > 0 */

    if (0U == pHandle->hPhaseRemainingTicks)
    {
      if (pHandle->pCurrentPhaseParams != MC_NULL)
      {
        /* If it becomes zero the current phase has been completed.*/
        /* Gives the next command to STC and VSS.*/
        uint16_t hPulse = pHandle->pCurrentPhaseParams->hFinalPulse * pHandle->PulseUpdateFactor / 10;
        (void)STC_ExecRamp(pHandle->pSTC, hPulse,
                           (uint32_t)(pHandle->pCurrentPhaseParams->hDurationms));
         
        VSS_SetMecAcceleration(pHandle->pVSS,
                               pHandle->pCurrentPhaseParams->hFinalMecSpeedUnit * pHandle->hDirection,
                               pHandle->pCurrentPhaseParams->hDurationms);

        /* Compute hPhaseRemainingTicks.*/
        pHandle->hPhaseRemainingTicks = (uint16_t)((((uint32_t)pHandle->pCurrentPhaseParams->hDurationms)
                                                  * ((uint32_t)pHandle->hRUCFrequencyHz)) / 1000U );
        pHandle->hPhaseRemainingTicks++;

        /*Set the next phases parameter pointer.*/
        pHandle->pCurrentPhaseParams = pHandle->pCurrentPhaseParams->pNext; //cstat !MISRAC2012-Rule-11.5

        /*Increases the rev up stages counter.*/
        pHandle->bStageCnt++;
      }
      else
      {
        retVal = false;
      }
    }
  }
  return (retVal);
}



/*
  * @brief  It is used to check if this stage is used for align motor.
  * @param  this related object of class CRUC.
  * @retval Returns 1 if the alignment is correct otherwise it returns 0
  */
uint8_t RUC_IsAlignStageNow(RevUpCtrl_Handle_t *pHandle)
{
  uint8_t align_flag = 0;

  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    int16_t speed;
    speed = RUC_GetPhaseFinalMecSpeed01Hz(pHandle, pHandle->bStageCnt);
    if (0 == speed)
    {
      align_flag = 1;
    }
  }
  return (align_flag);
}

/*
  * @brief  Provide current state of revup controller procedure.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  *  @retval Boolean set to true when entire revup phases have been completed.
  */
__weak bool RUC_Completed(RevUpCtrl_Handle_t *pHandle)
{
  bool retVal = false;

  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    if (MC_NULL == pHandle->pCurrentPhaseParams)
    {
      retVal = true;
    }
  }
  return (retVal);
}

/*
  * @brief  Allow to exit from RevUp process at the current rotor speed.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  */
__weak void RUC_Stop(RevUpCtrl_Handle_t *pHandle)
{
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    VirtualSpeedSensor_Handle_t *pVSS = pHandle->pVSS;
    pHandle->pCurrentPhaseParams = MC_NULL;
    pHandle->hPhaseRemainingTicks = 0U;
    VSS_SetMecAcceleration(pVSS, SPD_GetAvrgMecSpeedUnit(&pVSS->_Super), 0U);
  }
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif

/*
  * @brief  Check that alignment and first acceleration stage are completed.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  *  @retval Boolean set to true when first acceleration stage has been reached.
  */
__weak bool RUC_FirstAccelerationStageReached( RevUpCtrl_Handle_t * pHandle)
{
  bool retVal = false;
  VirtualSpeedSensor_Handle_t *pVSS = pHandle->pVSS;
  int16_t hSpeed;
  
  hSpeed = SPD_GetAvrgMecSpeedUnit(&pVSS->_Super);
  if (hSpeed < 0) hSpeed = -hSpeed;
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    if (pHandle->bStageCnt >= pHandle->bFirstAccelerationStage)
    {
      retVal = true;
    }
  }
  return (retVal);
}

/*
  * @brief  Allow to modify duration of a selected phase.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  * @param  bPhase: RevUp phase where new duration shall be modified.
  *         This parameter must be a number between 0 and 6.
  * @param  hDurationms: new duration value required for associated phase.
  *         This parameter must be set in millisecond.
  */
__weak void RUC_SetPhaseDurationms(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase, uint16_t hDurationms)
{
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    pHandle->ParamsData[bPhase].hDurationms = hDurationms;
  }
}

/*
  * @brief  Allow to modify targeted mechanical speed of a selected phase.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  * @param  bPhase: RevUp phase where new mechanical speed shall be modified.
  *         This parameter must be a number between 0 and 6.
  * @param  hFinalMecSpeedUnit: new targeted mechanical speed.
  *         This parameter must be expressed in 0.1Hz.
  */
__weak void RUC_SetPhaseFinalMecSpeedUnit(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase, int16_t hFinalMecSpeedUnit)
{
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    pHandle->ParamsData[bPhase].hFinalMecSpeedUnit = hFinalMecSpeedUnit;
  }
}

/**
  * @brief  Allow to modify targeted PWM counter pulse of a selected phase.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  * @param  bPhase: RevUp phase where new the motor torque shall be modified.
  *         This parameter must be a number between 0 and 6.
  * @param  hFinalPulse: new targeted motor torque.
  */
__weak void RUC_SetPhaseFinalPulse(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase, uint16_t hFinalPulse)
{
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    pHandle->ParamsData[bPhase].hFinalPulse = hFinalPulse;
  }
}

/*
  * @brief  Allow to configure a revUp phase.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  *  @retval Returns boolean set to true
  */
__weak bool RUC_SetPhase(RevUpCtrl_Handle_t *pHandle, uint8_t phaseNumber, RevUpCtrl_PhaseParams_t *phaseData)
{
  bool retValue = true;

  if ((MC_NULL == pHandle) || (MC_NULL == phaseData))
  {
    retValue = false;
  }
  else
  {
    pHandle->ParamsData[phaseNumber].hFinalPulse = phaseData->hFinalPulse;
    pHandle->ParamsData[phaseNumber].hFinalMecSpeedUnit = phaseData->hFinalMecSpeedUnit;
    pHandle->ParamsData[phaseNumber].hDurationms = phaseData->hDurationms;
  }
  return (retValue);
}

/*
  * @brief  Allow to read duration set in selected phase.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  * @param  bPhase: RevUp phase from where duration is read.
  *         This parameter must be a number between 0 and 6.
  *  @retval Returns duration used in selected phase.
  */
__weak uint16_t RUC_GetPhaseDurationms(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase)
{
  return ((MC_NULL == pHandle) ? 0U : (uint16_t)pHandle->ParamsData[bPhase].hDurationms);
}

/*
  * @brief  Allow to read targeted rotor speed set in selected phase.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  * @param  bPhase: RevUp phase from where targeted rotor speed is read.
  *         This parameter must be a number between 0 and 6.
  *  @retval Returns targeted rotor speed set in selected phase.
  */
__weak int16_t RUC_GetPhaseFinalMecSpeedUnit(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase)
{
  return ((MC_NULL == pHandle) ? 0 : (int16_t)pHandle->ParamsData[bPhase].hFinalMecSpeedUnit);
}

/**
  * @brief  Allow to read targeted PWM counter pulse set in selected phase.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  * @param  bPhase: RevUp phase from where targeted motor torque is read.
  *         This parameter must be a number between 0 and 6.
  *  @retval Returns targeted motor torque set in selected phase.
  */
__weak int16_t RUC_GetPhaseFinalPulse(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase)
{
  return ((MC_NULL == pHandle) ? 0 : (int16_t)pHandle->ParamsData[bPhase].hFinalPulse);
}

/*
  * @brief  Allow to read total number of programmed phases.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  *  @retval Returns number of phases relative to the programmed revup.
  */
__weak uint8_t RUC_GetNumberOfPhases(RevUpCtrl_Handle_t *pHandle)
{
  return ((MC_NULL == pHandle) ? 0U : (uint8_t)pHandle->bPhaseNbr);
}

/*
  * @brief  Allow to read a programmed phase.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  *  @retval Returns number of phases relative to the programmed revup.
  */
__weak bool RUC_GetPhase(RevUpCtrl_Handle_t *pHandle, uint8_t phaseNumber, RevUpCtrl_PhaseParams_t *phaseData)
{
  bool retValue = true;

  if ((MC_NULL == pHandle) || (MC_NULL == phaseData))
  {
    retValue = false;
  }
  else
  {
    phaseData->hFinalPulse = (int16_t)pHandle->ParamsData[phaseNumber].hFinalPulse;
    phaseData->hFinalMecSpeedUnit = (int16_t)pHandle->ParamsData[phaseNumber].hFinalMecSpeedUnit;
    phaseData->hDurationms = (uint16_t)pHandle->ParamsData[phaseNumber].hDurationms;
  }
  return (retValue);
}

/**
  * @brief  Allow to read spinning direction of the motor
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  *  @retval Returns direction of the motor.
  */
__weak int16_t RUC_GetDirection(RevUpCtrl_Handle_t *pHandle)
{
  return ((MC_NULL == pHandle) ? 0U : (int16_t)pHandle->hDirection);
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
