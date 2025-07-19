/**
  ******************************************************************************
  * @file    revup_ctrl.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the Rev-Up Control component of the Motor Control SDK:
  *
  *          * Main Rev-Up procedure to execute programmed phases
  *          * On the Fly (OTF)
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
  * @ingroup RevUpCtrlFOC
  */

/* Includes ------------------------------------------------------------------*/
#include "revup_ctrl.h"

/** @addtogroup MCSDK
  * @{
  */


/** @defgroup RevUpCtrl Rev-Up Control
  * @brief Rev-Up Control component of the Motor Control SDK
  *
  * Used to start up the motor with a set of pre-programmed phases.
  *
  * The number of phases of the Rev-Up procedure range is from 0 to 5.
  * The Rev-Up controller must be called at speed loop frequency.
  *
  * @{
  */
  
  
  

/** @defgroup RevUpCtrlFOC FOC Rev-Up Control component
  * @brief Rev-Up control component used to start motor driven with 
  *        the Field Oriented Control technique
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
#ifdef NULL_PTR_REV_UP_CTL
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    hRetVal = pHandle->ParamsData[bPhase].hFinalMecSpeedUnit;
#ifdef NULL_PTR_REV_UP_CTL
  }
#endif
  return (hRetVal);
}


/**
  * @brief  Initialize and configure the FOC RevUpCtrl Component
  * @param  pHandle: Pointer on Handle structure of FOC RevUp controller.
  * @param  pSTC: Pointer on speed and torque controller structure.
  * @param  pVSS: Pointer on virtual speed sensor structure.
  * @param  pSNSL: Pointer on sensorless state observer structure.
  * @param  pPWM: Pointer on the PWM structure.
  */
__weak void RUC_Init(	RevUpCtrl_Handle_t *pHandle,
						SpeednTorqCtrl_Handle_t *pSTC,
						VirtualSpeedSensor_Handle_t *pVSS,
						STO_Handle_t *pSNSL,
						PWMC_Handle_t *pPWM)
{
#ifdef NULL_PTR_REV_UP_CTL
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    RevUpCtrl_PhaseParams_t *pRUCPhaseParams = &pHandle->ParamsData[0];
    uint8_t bPhase = 0U;

    pHandle->pSTC = pSTC;
    pHandle->pVSS = pVSS;
    pHandle->pSNSL = pSNSL;
    pHandle->pPWM = pPWM;
    pHandle->OTFSCLowside = false;
    pHandle->EnteredZone1 = false;

    while ((pRUCPhaseParams != MC_NULL) && (bPhase < RUC_MAX_PHASE_NUMBER))
    {
      /* Dump HF data for now HF data are forced to 16 bits*/
      pRUCPhaseParams = (RevUpCtrl_PhaseParams_t *)pRUCPhaseParams->pNext;  //cstat !MISRAC2012-Rule-11.5
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

      pHandle->bResetPLLTh = (uint8_t)((RUC_OTF_PLL_RESET_TIMEOUT * pHandle->hRUCFrequencyHz) / 1000U);
    }
#ifdef NULL_PTR_REV_UP_CTL
  }
#endif
}

/**
  * @brief  Initialize internal FOC RevUp controller state.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  * @param  hMotorDirection: Rotor rotation direction.
  *         This parameter must be -1 or +1.
  */
__weak void RUC_Clear(RevUpCtrl_Handle_t *pHandle, int16_t hMotorDirection)
{
#ifdef NULL_PTR_REV_UP_CTL
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    VirtualSpeedSensor_Handle_t *pVSS = pHandle->pVSS;
    SpeednTorqCtrl_Handle_t *pSTC = pHandle->pSTC;
    RevUpCtrl_PhaseParams_t *pPhaseParams = pHandle->ParamsData;

    pHandle->hDirection = hMotorDirection;
    pHandle->EnteredZone1 = false;

    /*Initializes the rev up stages counter.*/
    pHandle->bStageCnt = 0U;
    pHandle->bOTFRelCounter = 0U;
    pHandle->OTFSCLowside = false;

    /* Calls the clear method of VSS.*/
    VSS_Clear(pVSS);

    /* Sets the STC in torque mode.*/
    STC_SetControlMode(pSTC, MCM_TORQUE_MODE);

    /* Sets the mechanical starting angle of VSS.*/
    VSS_SetMecAngle(pVSS, pHandle->hStartingMecAngle * hMotorDirection);

    /* Sets to zero the starting torque of STC */
    (void)STC_ExecRamp(pSTC, 0, 0U);

    /* Gives the first command to STC and VSS.*/
    (void)STC_ExecRamp(pSTC, pPhaseParams->hFinalTorque * hMotorDirection, (uint32_t)(pPhaseParams->hDurationms));

    VSS_SetMecAcceleration(pVSS, pPhaseParams->hFinalMecSpeedUnit * hMotorDirection, pPhaseParams->hDurationms);

    /* Compute hPhaseRemainingTicks.*/
    pHandle->hPhaseRemainingTicks = (uint16_t)((((uint32_t)pPhaseParams->hDurationms)
                                              * ((uint32_t)pHandle->hRUCFrequencyHz))
                                              / 1000U );

    pHandle->hPhaseRemainingTicks++;

    /*Set the next phases parameter pointer.*/
    pHandle->pCurrentPhaseParams = (RevUpCtrl_PhaseParams_t *)pPhaseParams->pNext;  //cstat !MISRAC2012-Rule-11.5

    /*Timeout counter for PLL reset during OTF.*/
    pHandle->bResetPLLCnt = 0U;
#ifdef NULL_PTR_REV_UP_CTL
  }
#endif
}

/**
  * @brief  FOC Main Rev-Up controller procedure executing overall programmed phases and
  *         on-the-fly startup handling.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  *  @retval Boolean set to false when entire Rev-Up phases have been completed.
  */
__weak bool RUC_OTF_Exec(RevUpCtrl_Handle_t *pHandle)
{
  bool retVal = true;
#ifdef NULL_PTR_REV_UP_CTL
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    bool IsSpeedReliable;
    bool condition = false;

    if (pHandle->hPhaseRemainingTicks > 0u)
    {
      /* Decrease the hPhaseRemainingTicks.*/
      pHandle->hPhaseRemainingTicks--;

      /* OTF start-up */
      if (0U == pHandle->bStageCnt)
      {
        if (false ==  pHandle->EnteredZone1)
        {
          if (pHandle->pSNSL->pFctStoOtfResetPLL != MC_NULL)
          {
            pHandle->bResetPLLCnt++;
            if (pHandle->bResetPLLCnt > pHandle->bResetPLLTh)
            {
              pHandle->pSNSL->pFctStoOtfResetPLL(pHandle->pSNSL);
              pHandle->bOTFRelCounter = 0U;
              pHandle->bResetPLLCnt = 0U;
            }
          }

          IsSpeedReliable = pHandle->pSNSL->pFctSTO_SpeedReliabilityCheck(pHandle->pSNSL);

          if (IsSpeedReliable)
          {
            if (pHandle->bOTFRelCounter < 127U)
            {
              pHandle->bOTFRelCounter++;
            }
          }
          else
          {
            pHandle->bOTFRelCounter = 0U;
          }

          if (pHandle->pSNSL->pFctStoOtfResetPLL != MC_NULL)
          {
            if (pHandle->bOTFRelCounter == (pHandle->bResetPLLTh >> 1))
            {
              condition = true;
            }
          }
          else
          {
            if (127U == pHandle->bOTFRelCounter)
            {
              condition = true;
            }
          }

          if (true == condition)
          {
            bool bCollinearSpeed = false;
            int16_t hObsSpeedUnit = SPD_GetAvrgMecSpeedUnit(pHandle->pSNSL->_Super);
            int16_t hObsSpeedUnitAbsValue =
                    ((hObsSpeedUnit < 0) ? (-hObsSpeedUnit) : (hObsSpeedUnit)); /* hObsSpeedUnit absolute value */

            if (pHandle->hDirection > 0)
            {
              if (hObsSpeedUnit > 0)
              {
                bCollinearSpeed = true; /* actual and reference speed are collinear*/
              }
            }
            else
            {
              if (hObsSpeedUnit < 0)
              {
                bCollinearSpeed = true; /* actual and reference speed are collinear*/
              }
            }

            if (false == bCollinearSpeed)
            {
              /*reverse speed management*/
              pHandle->bOTFRelCounter = 0U;
            }
            else /*speeds are collinear*/
            {
              if ((uint16_t)(hObsSpeedUnitAbsValue) > pHandle->hMinStartUpValidSpeed)
              {
                /* startup end, go to run */
                pHandle->pSNSL->pFctForceConvergency1(pHandle->pSNSL);
                pHandle->EnteredZone1 = true;
              }
              else if ((uint16_t)(hObsSpeedUnitAbsValue) > pHandle->hMinStartUpFlySpeed)
              {
                /* synch with startup*/
                /* nearest phase search*/
                int16_t hOldFinalMecSpeedUnit = 0;
                int16_t hOldFinalTorque = 0;
                int32_t wDeltaSpeedRevUp;
                int32_t wDeltaTorqueRevUp;
                bool bError = false;
                VSS_SetCopyObserver(pHandle->pVSS);
                pHandle->pSNSL->pFctForceConvergency2(pHandle->pSNSL);

                if (MC_NULL == pHandle->pCurrentPhaseParams)
                {
                  bError = true;
                  pHandle->hPhaseRemainingTicks = 0U;
                }
                else
                {
                  while (pHandle->pCurrentPhaseParams->hFinalMecSpeedUnit < hObsSpeedUnitAbsValue)
                  {
                    if (pHandle->pCurrentPhaseParams->pNext == MC_NULL)
                    {
                      /* sets for Revup fail error*/
                      bError = true;
                      pHandle->hPhaseRemainingTicks = 0U;
                      break;
                    }
                    else
                    {
                      /* skips this phase*/
                      hOldFinalMecSpeedUnit = pHandle->pCurrentPhaseParams->hFinalMecSpeedUnit;
                      hOldFinalTorque = pHandle->pCurrentPhaseParams->hFinalTorque;
                      //cstat !MISRAC2012-Rule-11.5
                      pHandle->pCurrentPhaseParams = (RevUpCtrl_PhaseParams_t *)pHandle->pCurrentPhaseParams->pNext;
                      pHandle->bStageCnt++;
                    }
                  }
                }
                if (false == bError)
                {
                  /* calculation of the transition phase from OTF to standard revup */
                  int16_t hTorqueReference;

                  wDeltaSpeedRevUp = ((int32_t)(pHandle->pCurrentPhaseParams->hFinalMecSpeedUnit))
                                   - ((int32_t)(hOldFinalMecSpeedUnit));
                  wDeltaTorqueRevUp = ((int32_t)(pHandle->pCurrentPhaseParams->hFinalTorque))
                                    - ((int32_t)(hOldFinalTorque));

                  if ((int32_t)0 == wDeltaSpeedRevUp)
                  {
                    /* Nothing to do */
                  }
                  else
                  {
                    hTorqueReference = (int16_t)((((int32_t)hObsSpeedUnit) * wDeltaTorqueRevUp) / wDeltaSpeedRevUp)
                                     + hOldFinalTorque;

                    (void)STC_ExecRamp(pHandle->pSTC, pHandle->hDirection * hTorqueReference, 0U);
                  }

                  pHandle->hPhaseRemainingTicks = 1U;

                  pHandle->pCurrentPhaseParams = &pHandle->OTFPhaseParams;

                  pHandle->bStageCnt = 6U;
                } /* no MC_NULL error */
              } /* speed > MinStartupFly */
              else
              {
              }
            } /* speeds are collinear */
          } /* speed is reliable */
        }/*EnteredZone1 1 is false */
        else
        {
          pHandle->pSNSL->pFctForceConvergency1(pHandle->pSNSL);
        }
      } /*stage 0*/
    } /* hPhaseRemainingTicks > 0 */

    if (0U == pHandle->hPhaseRemainingTicks)
    {
      if (pHandle->pCurrentPhaseParams != MC_NULL)
      {
        if (0U == pHandle->bStageCnt)
        {
          /*end of OTF*/
          PWMC_SwitchOffPWM(pHandle->pPWM);
          pHandle->OTFSCLowside = true;
          PWMC_TurnOnLowSides(pHandle->pPWM, 0u);
          pHandle->bOTFRelCounter = 0U;
        }
        else if (1U == pHandle->bStageCnt)
        {
          PWMC_SwitchOnPWM(pHandle->pPWM);
          pHandle->OTFSCLowside = false;
        }
        else
        {
        }

        /* If it becomes zero the current phase has been completed.*/
        /* Gives the next command to STC and VSS.*/
        (void)STC_ExecRamp(pHandle->pSTC, pHandle->pCurrentPhaseParams->hFinalTorque * pHandle->hDirection,
                           (uint32_t)(pHandle->pCurrentPhaseParams->hDurationms));

        VSS_SetMecAcceleration(pHandle->pVSS,
                               pHandle->pCurrentPhaseParams->hFinalMecSpeedUnit * pHandle->hDirection,
                               pHandle->pCurrentPhaseParams->hDurationms);

        /* Compute hPhaseRemainingTicks.*/
        pHandle->hPhaseRemainingTicks = (uint16_t)((((uint32_t)pHandle->pCurrentPhaseParams->hDurationms)
                                                 * (uint32_t)pHandle->hRUCFrequencyHz) / 1000U);
        pHandle->hPhaseRemainingTicks++;

        /*Set the next phases parameter pointer.*/
        pHandle->pCurrentPhaseParams = pHandle->pCurrentPhaseParams->pNext; //cstat !MISRAC2012-Rule-11.5

        /*Increases the rev up stages counter.*/
        pHandle->bStageCnt++;
      }
      else
      {
        if (pHandle->bStageCnt == (pHandle->bPhaseNbr - (uint8_t)1)) /* End of user programmed revup */
        {
          retVal = false;
        }
        else if (7U == pHandle->bStageCnt) /* End of first OTF runs */
        {
          pHandle->bStageCnt = 0U; /* Breaking state */
          pHandle->hPhaseRemainingTicks = 0U;
        }
        else
        {
          /* Nothing to do */
        }
      }
    }
#ifdef NULL_PTR_REV_UP_CTL
  }
#endif
  return (retVal);
}

/**
  * @brief  FOC Main Rev-Up controller procedure executing overall programmed phases.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  *  @retval Boolean set to false when entire Rev-Up phases have been completed.
  */
__weak bool RUC_Exec(RevUpCtrl_Handle_t *pHandle)
{
  bool retVal = true;
#ifdef NULL_PTR_REV_UP_CTL
  if (MC_NULL == pHandle)
  {
    retVal = false;
  }
  else
  {
#endif
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
        (void)STC_ExecRamp(pHandle->pSTC, pHandle->pCurrentPhaseParams->hFinalTorque * pHandle->hDirection,
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
#ifdef NULL_PTR_REV_UP_CTL
  }
#endif
  return (retVal);
}

/**
  * @brief  It checks if this stage is used for align motor.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  * @retval Returns 1 if the alignment is correct otherwise it returns 0
  */
uint8_t RUC_IsAlignStageNow(RevUpCtrl_Handle_t *pHandle)
{
  uint8_t align_flag = 0;
#ifdef NULL_PTR_REV_UP_CTL
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    int16_t speed;
    speed = RUC_GetPhaseFinalMecSpeed01Hz(pHandle, pHandle->bStageCnt);
    if (0 == speed)
    {
      align_flag = 1;
    }
#ifdef NULL_PTR_REV_UP_CTL
  }
#endif
  return (align_flag);
}

/**
  * @brief  Provide current state of Rev-Up controller procedure.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  *  @retval Boolean set to true when entire Rev-Up phases have been completed.
  */
__weak bool RUC_Completed(RevUpCtrl_Handle_t *pHandle)
{
  bool retVal = false;
#ifdef NULL_PTR_REV_UP_CTL
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    if (MC_NULL == pHandle->pCurrentPhaseParams)
    {
      retVal = true;
    }
#ifdef NULL_PTR_REV_UP_CTL
  }
#endif
  return (retVal);
}

/**
  * @brief  Allow to exit from Rev-Up process at the current rotor speed.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  */
__weak void RUC_Stop(RevUpCtrl_Handle_t *pHandle)
{
#ifdef NULL_PTR_REV_UP_CTL
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    VirtualSpeedSensor_Handle_t *pVSS = pHandle->pVSS;
    pHandle->pCurrentPhaseParams = MC_NULL;
    pHandle->hPhaseRemainingTicks = 0U;
    VSS_SetMecAcceleration(pVSS, SPD_GetAvrgMecSpeedUnit(&pVSS->_Super), 0U);
#ifdef NULL_PTR_REV_UP_CTL
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
  * @brief  Check that alignment and first acceleration stage are completed.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  *  @retval Boolean set to true when first acceleration stage has been reached.
  */
__weak bool RUC_FirstAccelerationStageReached(RevUpCtrl_Handle_t *pHandle)
{
  bool retVal = false;
#ifdef NULL_PTR_REV_UP_CTL
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    if (pHandle->bStageCnt >= pHandle->bFirstAccelerationStage)
    {
      retVal = true;
    }
#ifdef NULL_PTR_REV_UP_CTL
  }
#endif
  return (retVal);
}

/**
  * @brief  Allow to modify duration of a selected phase.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  * @param  bPhase: RevUp phase where new duration shall be modified.
  *         This parameter must be a number between 0 and 6.
  * @param  hDurationms: new duration value required for associated phase.
  *         This parameter must be set in millisecond.
  */
__weak void RUC_SetPhaseDurationms(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase, uint16_t hDurationms)
{
#ifdef NULL_PTR_REV_UP_CTL
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->ParamsData[bPhase].hDurationms = hDurationms;
#ifdef NULL_PTR_REV_UP_CTL
  }
#endif
}

/**
  * @brief  Allow to modify targeted mechanical speed of a selected phase.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  * @param  bPhase: RevUp phase where new mechanical speed shall be modified.
  *         This parameter must be a number between 0 and 6.
  * @param  hFinalMecSpeedUnit: new targeted mechanical speed.
  *         This parameter must be expressed in 0.1Hz.
  */
__weak void RUC_SetPhaseFinalMecSpeedUnit(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase, int16_t hFinalMecSpeedUnit)
{
#ifdef NULL_PTR_REV_UP_CTL
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->ParamsData[bPhase].hFinalMecSpeedUnit = hFinalMecSpeedUnit;
#ifdef NULL_PTR_REV_UP_CTL
  }
#endif
}

/**
  * @brief  Allow to modify targeted the motor torque of a selected phase.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  * @param  bPhase: RevUp phase where new the motor torque shall be modified.
  *         This parameter must be a number between 0 and 6.
  * @param  hFinalTorque: new targeted motor torque.
  */
__weak void RUC_SetPhaseFinalTorque(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase, int16_t hFinalTorque)
{
#ifdef NULL_PTR_REV_UP_CTL
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->ParamsData[bPhase].hFinalTorque = hFinalTorque;
#ifdef NULL_PTR_REV_UP_CTL
  }
#endif
}

/**
  * @brief  Allow to configure a Rev-Up phase.
  * @param  pHandle: Pointer on Handle structure of Rev-Up controller.
  * @param  phaseNumber : number of phases relative to the programmed Rev-Up
  * @param  phaseData:
  * 			- RevUp phase where new motor torque shall be modified.
  *        			This parameter must be a number between 0 and 6.
  *        		- RevUp phase where new mechanical speed shall be modified.
  *         		This parameter must be a number between 0 and 6.
  *        		- New duration value required for associated phase in ms unit.
  *  @retval Boolean set to true
  */
__weak bool RUC_SetPhase(RevUpCtrl_Handle_t *pHandle, uint8_t phaseNumber, RevUpCtrl_PhaseParams_t *phaseData)
{
  bool retValue = true;
#ifdef NULL_PTR_REV_UP_CTL
  if ((MC_NULL == pHandle) || (MC_NULL == phaseData))
  {
    retValue = false;
  }
  else
  {
#endif
    pHandle->ParamsData[phaseNumber].hFinalTorque = phaseData->hFinalTorque;
    pHandle->ParamsData[phaseNumber].hFinalMecSpeedUnit = phaseData->hFinalMecSpeedUnit;
    pHandle->ParamsData[phaseNumber].hDurationms = phaseData->hDurationms;
#ifdef NULL_PTR_REV_UP_CTL
  }
#endif
  return (retValue);
}

/**
  * @brief  Allow to read duration set in selected phase.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  * @param  bPhase: RevUp phase from where duration is read.
  *         This parameter must be a number between 0 and 6.
  *  @retval Returns duration used in selected phase in ms unit.
  */
__weak uint16_t RUC_GetPhaseDurationms(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase)
{
#ifdef NULL_PTR_REV_UP_CTL
  return ((MC_NULL == pHandle) ? 0U : (uint16_t)pHandle->ParamsData[bPhase].hDurationms);
#else
  return ((uint16_t)pHandle->ParamsData[bPhase].hDurationms);
#endif
}

/**
  * @brief  Allow to read targeted rotor speed set in selected phase.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  * @param  bPhase: RevUp phase from where targeted rotor speed is read.
  *         This parameter must be a number between 0 and 6.
  *  @retval Returns targeted rotor speed set for a selected phase.
  */
__weak int16_t RUC_GetPhaseFinalMecSpeedUnit(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase)
{
#ifdef NULL_PTR_REV_UP_CTL
  return ((MC_NULL == pHandle) ? 0 : (int16_t)pHandle->ParamsData[bPhase].hFinalMecSpeedUnit);
#else
  return ((int16_t)pHandle->ParamsData[bPhase].hFinalMecSpeedUnit);
#endif
}

/**
  * @brief  Allow to read targeted motor torque set in selected phase.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  * @param  bPhase: RevUp phase from where targeted motor torque is read.
  *         This parameter must be a number between 0 and 6.
  *  @retval Returns targeted motor torque set for a selected phase.
  */
__weak int16_t RUC_GetPhaseFinalTorque(RevUpCtrl_Handle_t *pHandle, uint8_t bPhase)
{
#ifdef NULL_PTR_REV_UP_CTL
  return ((MC_NULL == pHandle) ? 0 : (int16_t)pHandle->ParamsData[bPhase].hFinalTorque);
#else
  return ((int16_t)pHandle->ParamsData[bPhase].hFinalTorque);
#endif
}

/**
  * @brief  Allow to read total number of programmed phases.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  *  @retval Returns number of phases relative to the programmed Rev-Up.
  *
  */

__weak uint8_t RUC_GetNumberOfPhases(RevUpCtrl_Handle_t *pHandle)
{
#ifdef NULL_PTR_REV_UP_CTL
  return ((MC_NULL == pHandle) ? 0U : (uint8_t)pHandle->bPhaseNbr);
#else
  return ((uint8_t)pHandle->bPhaseNbr);
#endif
}

/**
  * @brief  Allow to read a programmed phase.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  * @param  phaseNumber : number of phases relative to the programmed Rev-Up
  * @param  phaseData:
  * 			- RevUp phase from where targeted rotor speed is read.
  *        			This parameter must be a number between 0 and 6.
  *        		- RevUp phase from where targeted motor torque is read.
  *         		This parameter must be a number between 0 and 6.
  *        		- Duration set in selected phase in ms unit.
  *  @retval Returns Boolean set to true value.
  */
__weak bool RUC_GetPhase(RevUpCtrl_Handle_t *pHandle, uint8_t phaseNumber, RevUpCtrl_PhaseParams_t *phaseData)
{
  bool retValue = true;
#ifdef NULL_PTR_REV_UP_CTL
  if ((MC_NULL == pHandle) || (MC_NULL == phaseData))
  {
    retValue = false;
  }
  else
  {
#endif
    phaseData->hFinalTorque = (int16_t)pHandle->ParamsData[phaseNumber].hFinalTorque;
    phaseData->hFinalMecSpeedUnit = (int16_t)pHandle->ParamsData[phaseNumber].hFinalMecSpeedUnit;
    phaseData->hDurationms = (uint16_t)pHandle->ParamsData[phaseNumber].hDurationms;
#ifdef NULL_PTR_REV_UP_CTL
  }
#endif
  return (retValue);
}

/**
  * @brief  Allow to read status of On The Fly (OTF) feature.
  * @param  pHandle: Pointer on Handle structure of RevUp controller.
  *  @retval Boolean set to true at the end of OTF processing.
  */
__weak bool RUC_Get_SCLowsideOTF_Status(RevUpCtrl_Handle_t *pHandle)
{
#ifdef NULL_PTR_REV_UP_CTL
  return ((MC_NULL == pHandle) ? false : pHandle->OTFSCLowside);
#else
  return (pHandle->OTFSCLowside);
#endif
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
