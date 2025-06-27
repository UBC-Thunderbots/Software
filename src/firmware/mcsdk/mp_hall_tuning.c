/**
  ******************************************************************************
  * @file    mp_hall_tuning.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   
  *
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics International N.V.
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
  */

/* Includes ------------------------------------------------------------------*/

#include "mp_one_touch_tuning.h"
#include "mp_self_com_ctrl.h"

#include "mc_config.h"
#include "mp_hall_tuning.h"
#include "hall_speed_pos_fdbk.h"
#include "mc_type.h"
#include "arm_math.h"
#include "main.h"

#include "motorcontrol.h"

#if defined(STM32_PROTECTED)
#include "STM32_Check.h"
#endif

//-----#define section begin-------
//-----#define section end---------

//-----global variables section begin-------
uint8_t bIndexPTC;
uint16_t ShiftAngleCnt;
Trig_Components Vector_PS;
//-----global variables section end---------


    
/**
  * @brief  Hall Tuning Initialization.       
  * @param  pHandleHT pointer on related component instance.
  * @retval none
  */
void HT_Init( HT_Handle_t * pHandleHT, bool pRST )
{
  MCI_State_t MCI_State = MCI_GetSTMState ( pHandleHT->pMCI );
  
  bIndexPTC = 0;
  ShiftAngleCnt = 0;
  
  pHandleHT->pHALL_M1->PinToComplement = NO_COMPLEMENT;
  pHandleHT->pHALL_M1->PhaseShift = 0;
  pHandleHT->pHALL_M1->AvrElSpeedDpp = 0;
  
  pHandleHT->userWantsToAbort = false;
  
  pHandleHT->flagState0 = false;
  pHandleHT->flagState1 = false;
  pHandleHT->H1Connected = false;
  pHandleHT->H2Connected = false;
  pHandleHT->H3Connected = false;
  pHandleHT->PTCWellPositioned = false;
  pHandleHT->waitHallFlagCompleted = false;
  pHandleHT->reliable = false;
  
  pHandleHT->bPinToComplement = NO_COMPLEMENT;
  pHandleHT->bNewH1 = 0;
  pHandleHT->bNewH2 = 0;
  pHandleHT->bNewH3 = 0;
  pHandleHT->previousH1 = 0;
  pHandleHT->previousH2 = 0;
  pHandleHT->previousH3 = 0;
  pHandleHT->bMechanicalWantedDirection = 0;
  
  pHandleHT->hPhaseShiftInstantaneous = 0;
  pHandleHT->hWaitRampCnt = 0;
  pHandleHT->hHallFlagCnt = 0;
  pHandleHT->bProgressPercentage = 0;
  
  pHandleHT->wSinMean = 0;
  pHandleHT->wCosMean = 0;
  
  pHandleHT->wSinSum1 = 0;
  pHandleHT->wCosSum1 = 0;
  pHandleHT->wSinSum2 = 0;
  pHandleHT->wCosSum2 = 0;
  pHandleHT->wSinSum3 = 0;
  pHandleHT->wCosSum3 = 0;
  pHandleHT->wSinSum4 = 0;
  pHandleHT->wCosSum4 = 0;
  pHandleHT->wSinSum5 = 0;
  pHandleHT->wCosSum5 = 0;
  pHandleHT->wSinSum6 = 0;
  pHandleHT->wCosSum6 = 0;
  
  if ( pRST == false )
  {
    pHandleHT->bPlacement = 0;
    pHandleHT->edgeAngleDirPos = true;
  
    pHandleHT->hPhaseShiftCircularMean = 0;
    pHandleHT->hPhaseShiftCircularMeanDeg = 0;
    pHandleHT->hPhaseShiftCircularMeanNeg = 0;
    pHandleHT->hPhaseShiftCircularMeanNegDeg = 0;
    
    pHandleHT->hPhaseShiftCircularMean5_1 = 0;
    pHandleHT->hPhaseShiftCircularMeanDeg5_1 = 0;
    pHandleHT->hPhaseShiftCircularMean1_3 = 0;
    pHandleHT->hPhaseShiftCircularMeanDeg1_3 = 0;
    pHandleHT->hPhaseShiftCircularMean3_2 = 0;
    pHandleHT->hPhaseShiftCircularMeanDeg3_2 = 0;
    pHandleHT->hPhaseShiftCircularMean2_6 = 0;
    pHandleHT->hPhaseShiftCircularMeanDeg2_6 = 0;
    pHandleHT->hPhaseShiftCircularMean6_4 = 0;
    pHandleHT->hPhaseShiftCircularMeanDeg6_4 = 0;
    pHandleHT->hPhaseShiftCircularMean4_5 = 0;
    pHandleHT->hPhaseShiftCircularMeanDeg4_5 = 0;
    
    pHandleHT->hPhaseShiftCircularMean5_4 = 0;
    pHandleHT->hPhaseShiftCircularMeanDeg5_4 = 0;
    pHandleHT->hPhaseShiftCircularMean4_6 = 0;
    pHandleHT->hPhaseShiftCircularMeanDeg4_6 = 0;
    pHandleHT->hPhaseShiftCircularMean6_2 = 0;
    pHandleHT->hPhaseShiftCircularMeanDeg6_2 = 0;
    pHandleHT->hPhaseShiftCircularMean2_3 = 0;
    pHandleHT->hPhaseShiftCircularMeanDeg2_3 = 0;
    pHandleHT->hPhaseShiftCircularMean3_1 = 0;
    pHandleHT->hPhaseShiftCircularMeanDeg3_1 = 0;
    pHandleHT->hPhaseShiftCircularMean1_5 = 0;
    pHandleHT->hPhaseShiftCircularMeanDeg1_5 = 0;
  
    pHandleHT->HT_Start = false;
    pHandleHT->directionAlreadyChosen = false;
    pHandleHT->userWantsToRestart = false;
    pHandleHT->sm_state = HT_IDLE;
    pHandleHT->shiftEdge_state = SHIFT_EDGE_IDLE;
    if (MC_GetMecSpeedReferenceMotor1() < 0)
    {
      MC_ProgramSpeedRampMotor1(-MC_GetMecSpeedReferenceMotor1(), 0);
    }
  }
  else
  {
    if ( pHandleHT->pOTT->bPI_Tuned == true )
    {
      pHandleHT->pMCI->State = STOP;
      if ( MCI_State == IDLE ) /* Buffered version of pMCI->State*/
      {
        pHandleHT->userWantsToRestart = false;
        pHandleHT->sm_state = HT_START_RAMP;
      }
    }
  }
}



void HT_MF( HT_Handle_t * pHandleHT )
{    
    MCI_State_t MCI_State = MCI_GetSTMState ( pHandleHT->pMCI );
    
    switch ( pHandleHT->sm_state )
    {
      case HT_IDLE:
      {
        if ( pHandleHT->pOTT->bPI_Tuned == true )
        {
          if ( MCI_State == IDLE ) 
          {
            if ( pHandleHT->HT_Start == true ) 
            {
              pHandleHT->hShiftAngleDepth = pHandleHT->pHALL_M1->_Super.bElToMecRatio * 60;
              pHandleHT->sm_state = HT_START_RAMP;
            }
          }
          else
          {}
        }
      }
      break;
      
      case HT_START_RAMP:
      {
        if ( pHandleHT->pHALL_M1->AvrElSpeedDpp == 0 )
        {
          if ( pHandleHT->edgeAngleDirPos )
          {
            if (MC_GetMecSpeedReferenceMotor1() > 0)
            {
              MC_ProgramSpeedRampMotor1(MC_GetMecSpeedReferenceMotor1(), 0);
            }
            else
            {
              MC_ProgramSpeedRampMotor1(-MC_GetMecSpeedReferenceMotor1(), 0);
            }
          }
          else
          {
            if (MC_GetMecSpeedReferenceMotor1() > 0)
            {
              MC_ProgramSpeedRampMotor1(-MC_GetMecSpeedReferenceMotor1(), 0);
            }
            else
            {
              MC_ProgramSpeedRampMotor1(MC_GetMecSpeedReferenceMotor1(), 0);
            }
          }
          
          MC_StartMotor1();
          pHandleHT->sm_state = HT_WAIT_RAMP;
        }
      }
      break;
      
      case HT_WAIT_RAMP:
      {
        if ( MC_HasRampCompletedMotor1() )
        {
          pHandleHT->hHallFlagCnt = 0;
          
          if ( pHandleHT->edgeAngleDirPos )
          {
            pHandleHT->sm_state = HT_CHECK_HALL_RELIABILITY;
          }
          else
          {
            pHandleHT->sm_state = HT_WAIT_STABILIZATION;
          }
        }
      }
      break;
      
      case HT_CHECK_HALL_RELIABILITY:
      { 
        if ( MCI_State == RUN && pHandleHT->waitHallFlagCompleted == true )  
        {
          if ( !pHandleHT->H1Connected && !pHandleHT->H2Connected && !pHandleHT->H3Connected )
          {
            pHandleHT->pMCI->State = STOP ;
            pHandleHT->sm_state = HT_ERROR_RELIABILITY;
            pHandleHT->pHALL_M1->AvrElSpeedDpp = 0;
            pHandleHT->hHallFlagCnt = 0;
            pHandleHT->reliable = false;
          }
          else if ( !pHandleHT->H1Connected || !pHandleHT->H2Connected || !pHandleHT->H3Connected )
          {
            pHandleHT->pMCI->State = STOP;
            pHandleHT->sm_state = HT_ERROR_RELIABILITY;
            pHandleHT->pHALL_M1->AvrElSpeedDpp = 0;
            pHandleHT->hHallFlagCnt = 0;
            pHandleHT->reliable = false;
          }
          else
          {
            pHandleHT->reliable = true;
            if ( pHandleHT->directionAlreadyChosen == false )
            {
              pHandleHT->waitHallFlagCompleted = false;
              pHandleHT->sm_state = HT_WAIT_USER_DIRECTION_CHOICE;
            }
            else
            {
              pHandleHT->waitHallFlagCompleted = false;
              pHandleHT->hHallFlagCnt = 0;
              pHandleHT->sm_state = HT_DETECTING_CONF;
            }
          }   
        }
        else if ( MCI_State == RUN )
        { 
          if ( pHandleHT->hHallFlagCnt == 0 )
          {
            pHandleHT->H3Connected = false;
            pHandleHT->H2Connected = false;
            pHandleHT->H1Connected = false;
            pHandleHT->flagState0 = false;
            pHandleHT->flagState1 = false;
          }
          else if ( pHandleHT->hHallFlagCnt >= HALL_FLAG_TIMEOUT ) {
            pHandleHT->waitHallFlagCompleted = true;
          }
          pHandleHT->hHallFlagCnt++;
        }
      }
      break;
      
      case HT_WAIT_USER_DIRECTION_CHOICE: //Pop-Up with 2 buttons : yes & no
      {
        if ( pHandleHT->bMechanicalWantedDirection == 1 ) 
        {
          pHandleHT->hHallFlagCnt = 0;
          pHandleHT->directionAlreadyChosen = true;
          pHandleHT->sm_state = HT_DETECTING_CONF;
        }
        else if ( pHandleHT->bMechanicalWantedDirection == 2 )
        {
          pHandleHT->directionAlreadyChosen = true;
          pHandleHT->pMCI->State = STOP;
          pHandleHT->sm_state = HT_WARNING_PHASES_NEED_SWAP;
        }
      }
      break;
      
      case HT_ERROR_RELIABILITY: //Pop-Up with 2 buttons : retry & abort
      {
        if ( pHandleHT->userWantsToRestart == true || pHandleHT->userWantsToAbort == true ) 
        {  
          pHandleHT->sm_state = HT_RST;
        }
      }
      break;
      
      case HT_ERROR_PINS_READING: //Pop-Up with 2 buttons : retry & abort
      {
        if ( pHandleHT->userWantsToRestart == true || pHandleHT->userWantsToAbort == true ) 
        {
          pHandleHT->sm_state = HT_RST;
        }
      }
      break;
      
      case HT_WARNING_PHASES_NEED_SWAP: //Pop-Up with 2 buttons : retry & abort
      {
        if ( pHandleHT->userWantsToRestart == true || pHandleHT->userWantsToAbort == true ) 
        {  
          pHandleHT->sm_state = HT_RST;
        }
      }
      break;
      
      case HT_DETECTING_CONF:
      {
        if ( MCI_State == RUN && pHandleHT->waitHallFlagCompleted == true )
        {
          if ( pHandleHT->flagState0 == 1 && pHandleHT->flagState1 == 1 ) 
          {
            pHandleHT->bPlacement = 60;
            pHandleHT->sm_state = HT_DETERMINE_PTC;
          }
          else 
          {
            if (pHandleHT->bPlacement == 0)
            {
              pHandleHT->bPlacement = 120;
              pHandleHT->bPinToComplement = NO_COMPLEMENT;
            }
            else
            {
              if ( pHandleHT->bPinToComplement == H2 )
              {
                pHandleHT->PTCWellPositioned = true;
              }
            }
            pHandleHT->sm_state = HT_DETECTING_SWAP;
          }
        }  
        else if ( MCI_State == RUN )
        {
          if ( pHandleHT->hHallFlagCnt == 0 )
          {
            pHandleHT->flagState0 = false;
            pHandleHT->flagState1 = false;
          }
          else if ( pHandleHT->hHallFlagCnt >= HALL_FLAG_TIMEOUT ) {
            pHandleHT->waitHallFlagCompleted = true;
          }
          pHandleHT->hHallFlagCnt++;
        }
      }
      break;
      
      case HT_DETERMINE_PTC:
      {
        if ( MCI_State == RUN )  
        {
          switch (bIndexPTC)
          {  
            case 0:
            {
              pHandleHT->pHALL_M1->PinToComplement = H1;
              pHandleHT->bPinToComplement = H1;
              bIndexPTC++;
              pHandleHT->hHallFlagCnt = 0;
              pHandleHT->waitHallFlagCompleted = false;
              pHandleHT->sm_state = HT_DETECTING_CONF;
            }
            break;
            case 1:
            {
              pHandleHT->pHALL_M1->PinToComplement = H2;
              pHandleHT->bPinToComplement = H2;
              bIndexPTC++;
              pHandleHT->hHallFlagCnt = 0;
              pHandleHT->waitHallFlagCompleted = false;
              pHandleHT->sm_state = HT_DETECTING_CONF;
            }
            break;
            case 2:
            {
              pHandleHT->pHALL_M1->PinToComplement = H3;
              pHandleHT->bPinToComplement = H3;
              pHandleHT->hHallFlagCnt = 0;
              pHandleHT->waitHallFlagCompleted = false;
              pHandleHT->sm_state = HT_DETECTING_CONF;
            }
            break;
            default:
            break;
          }
        }
      }
      break;
      
      case HT_DETECTING_SWAP:
      {
        if ( MCI_State == RUN )  
        {
          if ( pHandleHT->bPlacement == 60 && pHandleHT->PTCWellPositioned == false ) 
          {
            if ( pHandleHT->bPinToComplement == H1 )
            {
              if ( pHandleHT->pHALL_M1->AvrElSpeedDpp < 0 ) 
              {
                pHandleHT->bNewH1 = 2;
                pHandleHT->bNewH2 = 1;
                pHandleHT->bNewH3 = 3;
              }
              else
              {
                pHandleHT->bNewH1 = 3;
                pHandleHT->bNewH2 = 1;
                pHandleHT->bNewH3 = 2;
              }
            }
            else
            {
              if ( pHandleHT->pHALL_M1->AvrElSpeedDpp < 0 ) 
              {
                pHandleHT->bNewH1 = 1;
                pHandleHT->bNewH2 = 3;
                pHandleHT->bNewH3 = 2;
              }
              else
              {
                pHandleHT->bNewH1 = 2;
                pHandleHT->bNewH2 = 3;
                pHandleHT->bNewH3 = 1;
              }
            }
            pHandleHT->pMCI->State=STOP;
                
            pHandleHT->sm_state = HT_ERROR_PINS_READING;
          }
          else if ( pHandleHT->bPlacement == 60 && pHandleHT->PTCWellPositioned == true )
          {
            if ( pHandleHT->pHALL_M1->AvrElSpeedDpp < 0 ) 
            {
              pHandleHT->pMCI->State=STOP;
              
              pHandleHT->bNewH1 = 3;
              pHandleHT->bNewH2 = 2;
              pHandleHT->bNewH3 = 1;
              
              pHandleHT->sm_state = HT_ERROR_PINS_READING;
            }
            else
            {
              pHandleHT->bNewH1 = 1;
              pHandleHT->bNewH2 = 2;
              pHandleHT->bNewH3 = 3;
              
              pHandleHT->sm_state = HT_WAIT_STABILIZATION;
            }
          }
          else if ( pHandleHT->bPlacement == 120 )
          {
            if ( pHandleHT->pHALL_M1->AvrElSpeedDpp < 0 ) 
            {
              pHandleHT->pMCI->State=STOP;
            
              pHandleHT->bNewH1 = 2;
              pHandleHT->bNewH2 = 1;
              pHandleHT->bNewH3 = 3;
              
              pHandleHT->sm_state = HT_ERROR_PINS_READING;
            }
            else
            {
              pHandleHT->bNewH1 = 1;
              pHandleHT->bNewH2 = 2;
              pHandleHT->bNewH3 = 3;
              
              pHandleHT->sm_state = HT_WAIT_STABILIZATION;
            }
          }
        }
      }
      break;
       
      case HT_WAIT_STABILIZATION:
      {
        if ( MCI_State == RUN && pHandleHT->pHALL_M1->AvrElSpeedDpp != 0 )
        {
          if ( pHandleHT->hWaitRampCnt < WAIT_RAMP_TIMEOUT )
          {
            pHandleHT->bProgressPercentage = (pHandleHT->hWaitRampCnt * 100) / WAIT_RAMP_TIMEOUT;
            pHandleHT->hWaitRampCnt++;
          }
          else
          {
            pHandleHT->sm_state = HT_DETECTING_EDGE;
          }
        }
      }
      break;
      
      case HT_CALC_EDGE_ANGLE:
      {
                            
        Vector_PS = MCM_Trig_Functions ( pHandleHT->hPhaseShiftInstantaneous );
        
        switch (pHandleHT->shiftEdge_state) {
          case SHIFT_EDGE_1:
          {
            pHandleHT->wSinSum1 += Vector_PS.hSin;
            pHandleHT->wCosSum1 += Vector_PS.hCos;
          }
          break;
          
          case SHIFT_EDGE_2:
          {
            pHandleHT->wSinSum2 += Vector_PS.hSin;
            pHandleHT->wCosSum2 += Vector_PS.hCos;
          }
          break;
        
          case SHIFT_EDGE_3:
          {
            pHandleHT->wSinSum3 += Vector_PS.hSin;
            pHandleHT->wCosSum3 += Vector_PS.hCos;
          }
          break;
        
          case SHIFT_EDGE_4:
          {
            pHandleHT->wSinSum4 += Vector_PS.hSin;
            pHandleHT->wCosSum4 += Vector_PS.hCos;
          }
          break;
        
          case SHIFT_EDGE_5:
          {
            pHandleHT->wSinSum5 += Vector_PS.hSin;
            pHandleHT->wCosSum5 += Vector_PS.hCos;
          }
          break;
        
          case SHIFT_EDGE_6:
          {
            pHandleHT->wSinSum6 += Vector_PS.hSin;
            pHandleHT->wCosSum6 += Vector_PS.hCos;
          }
          break;
        
          default:
          break;
        }
          
        pHandleHT->bProgressPercentage = ((ShiftAngleCnt * 100) / pHandleHT->hShiftAngleDepth);
        ShiftAngleCnt++;
        
        if ( ShiftAngleCnt < pHandleHT->hShiftAngleDepth )
        {
          pHandleHT->sm_state = HT_DETECTING_EDGE;
        }
        else //end of acquisition, we can compute the result
        {
          pHandleHT->shiftEdge_state = SHIFT_EDGE_END;
          pHandleHT->sm_state = HT_EDGE_COMPUTATION;
        }
      
      }
      break;
      
      case HT_EDGE_COMPUTATION:
      {          
        pHandleHT->wCosSum1 /= pHandleHT->hShiftAngleDepth/6;
        pHandleHT->wSinSum1 /= pHandleHT->hShiftAngleDepth/6;
        pHandleHT->wCosSum2 /= pHandleHT->hShiftAngleDepth/6;
        pHandleHT->wSinSum2 /= pHandleHT->hShiftAngleDepth/6;
        pHandleHT->wCosSum3 /= pHandleHT->hShiftAngleDepth/6;
        pHandleHT->wSinSum3 /= pHandleHT->hShiftAngleDepth/6;
        pHandleHT->wCosSum4 /= pHandleHT->hShiftAngleDepth/6;
        pHandleHT->wSinSum4 /= pHandleHT->hShiftAngleDepth/6;
        pHandleHT->wCosSum5 /= pHandleHT->hShiftAngleDepth/6;
        pHandleHT->wSinSum5 /= pHandleHT->hShiftAngleDepth/6;
        pHandleHT->wCosSum6 /= pHandleHT->hShiftAngleDepth/6;
        pHandleHT->wSinSum6 /= pHandleHT->hShiftAngleDepth/6;
        
        //positive direction
        if( pHandleHT->edgeAngleDirPos ) {
          //Phase shift at state 5 for positive direction
          pHandleHT->hPhaseShiftCircularMean = MCM_PhaseComputation(pHandleHT->wCosSum5, pHandleHT->wSinSum5);
            
          pHandleHT->hPhaseShiftCircularMeanDeg = pHandleHT->hPhaseShiftCircularMean * 180 / 32768;
          if ( pHandleHT->hPhaseShiftCircularMeanDeg < 0)
          {
            pHandleHT->hPhaseShiftCircularMeanDeg += 360;
          }
          
          //Shift between each state
          //cos(a - b) = cos a cos b + sin a sin b
          pHandleHT->wCosMean = (pHandleHT->wCosSum1 * pHandleHT->wCosSum5) + (pHandleHT->wSinSum1 * pHandleHT->wSinSum5);
          //sin(a - b) = sin a cos b - sin b cos a
          pHandleHT->wSinMean = (pHandleHT->wSinSum1 * pHandleHT->wCosSum5) - (pHandleHT->wSinSum5 * pHandleHT->wCosSum1);
          
          pHandleHT->hPhaseShiftCircularMean5_1 = MCM_PhaseComputation( pHandleHT->wCosMean, pHandleHT->wSinMean);
          pHandleHT->hPhaseShiftCircularMeanDeg5_1 = pHandleHT->hPhaseShiftCircularMean5_1 * 180 / 32768;
          
          
          //cos(a - b) = cos a cos b + sin a sin b
          pHandleHT->wCosMean = (pHandleHT->wCosSum3 * pHandleHT->wCosSum1) + (pHandleHT->wSinSum3 * pHandleHT->wSinSum1);
          //sin(a - b) = sin a cos b - sin b cos a
          pHandleHT->wSinMean = (pHandleHT->wSinSum3 * pHandleHT->wCosSum1) - (pHandleHT->wSinSum1 * pHandleHT->wCosSum3);
          
          pHandleHT->hPhaseShiftCircularMean1_3 = MCM_PhaseComputation( pHandleHT->wCosMean, pHandleHT->wSinMean);
          pHandleHT->hPhaseShiftCircularMeanDeg1_3 = pHandleHT->hPhaseShiftCircularMean1_3 * 180 / 32768;
          
          
          //cos(a - b) = cos a cos b + sin a sin b
          pHandleHT->wCosMean = (pHandleHT->wCosSum2 * pHandleHT->wCosSum3) + (pHandleHT->wSinSum2 * pHandleHT->wSinSum3);
          //sin(a - b) = sin a cos b - sin b cos a
          pHandleHT->wSinMean = (pHandleHT->wSinSum2 * pHandleHT->wCosSum3) - (pHandleHT->wSinSum3 * pHandleHT->wCosSum2);
          
          pHandleHT->hPhaseShiftCircularMean3_2 = MCM_PhaseComputation( pHandleHT->wCosMean, pHandleHT->wSinMean);
          pHandleHT->hPhaseShiftCircularMeanDeg3_2 = pHandleHT->hPhaseShiftCircularMean3_2 * 180 / 32768;
          
          
          //cos(a - b) = cos a cos b + sin a sin b
          pHandleHT->wCosMean = (pHandleHT->wCosSum6 * pHandleHT->wCosSum2) + (pHandleHT->wSinSum6 * pHandleHT->wSinSum2);
          //sin(a - b) = sin a cos b - sin b cos a
          pHandleHT->wSinMean = (pHandleHT->wSinSum6 * pHandleHT->wCosSum2) - (pHandleHT->wSinSum2 * pHandleHT->wCosSum6);
          
          pHandleHT->hPhaseShiftCircularMean2_6 = MCM_PhaseComputation( pHandleHT->wCosMean, pHandleHT->wSinMean);
          pHandleHT->hPhaseShiftCircularMeanDeg2_6 = pHandleHT->hPhaseShiftCircularMean2_6 * 180 / 32768;
          
          
          //cos(a - b) = cos a cos b + sin a sin b
          pHandleHT->wCosMean = (pHandleHT->wCosSum4 * pHandleHT->wCosSum6) + (pHandleHT->wSinSum4 * pHandleHT->wSinSum6);
          //sin(a - b) = sin a cos b - sin b cos a
          pHandleHT->wSinMean = (pHandleHT->wSinSum4 * pHandleHT->wCosSum6) - (pHandleHT->wSinSum6 * pHandleHT->wCosSum4);
          
          pHandleHT->hPhaseShiftCircularMean6_4 = MCM_PhaseComputation( pHandleHT->wCosMean, pHandleHT->wSinMean);
          pHandleHT->hPhaseShiftCircularMeanDeg6_4 = pHandleHT->hPhaseShiftCircularMean6_4 * 180 / 32768;
          
          
          //cos(a - b) = cos a cos b + sin a sin b
          pHandleHT->wCosMean = (pHandleHT->wCosSum5 * pHandleHT->wCosSum4) + (pHandleHT->wSinSum5 * pHandleHT->wSinSum4);
          //sin(a - b) = sin a cos b - sin b cos a
          pHandleHT->wSinMean = (pHandleHT->wSinSum5 * pHandleHT->wCosSum4) - (pHandleHT->wSinSum4 * pHandleHT->wCosSum5);
          
          pHandleHT->hPhaseShiftCircularMean4_5 = MCM_PhaseComputation( pHandleHT->wCosMean, pHandleHT->wSinMean);
          pHandleHT->hPhaseShiftCircularMeanDeg4_5 = pHandleHT->hPhaseShiftCircularMean4_5 * 180 / 32768;
          
          //transition for negative direction measurements
          ShiftAngleCnt = 0;  
          pHandleHT->edgeAngleDirPos = false;
          pHandleHT->userWantsToRestart = true;
          pHandleHT->sm_state = HT_RST;
        }
        //negative direction
        else
        {  
          //Phase shift at state 5 for negative direction
          pHandleHT->hPhaseShiftCircularMeanNeg = MCM_PhaseComputation(pHandleHT->wCosSum4, pHandleHT->wSinSum4);
            
          pHandleHT->hPhaseShiftCircularMeanNegDeg = pHandleHT->hPhaseShiftCircularMeanNeg * 180 / 32768;
          if ( pHandleHT->hPhaseShiftCircularMeanNegDeg < 0)
          {
            pHandleHT->hPhaseShiftCircularMeanNegDeg += 360;
          }
          
          //Shift between each state
          //cos(a - b) = cos a cos b + sin a sin b
          pHandleHT->wCosMean = (pHandleHT->wCosSum4 * pHandleHT->wCosSum5) + (pHandleHT->wSinSum4 * pHandleHT->wSinSum5);
          //sin(a - b) = sin a cos b - sin b cos a
          pHandleHT->wSinMean = (pHandleHT->wSinSum4 * pHandleHT->wCosSum5) - (pHandleHT->wSinSum5 * pHandleHT->wCosSum4);
          
          pHandleHT->hPhaseShiftCircularMean5_4 = MCM_PhaseComputation( pHandleHT->wCosMean, pHandleHT->wSinMean);
          pHandleHT->hPhaseShiftCircularMeanDeg5_4 = pHandleHT->hPhaseShiftCircularMean5_4 * 180 / 32768;
          
          
          //cos(a - b) = cos a cos b + sin a sin b
          pHandleHT->wCosMean = (pHandleHT->wCosSum6 * pHandleHT->wCosSum4) + (pHandleHT->wSinSum6 * pHandleHT->wSinSum4);
          //sin(a - b) = sin a cos b - sin b cos a
          pHandleHT->wSinMean = (pHandleHT->wSinSum6 * pHandleHT->wCosSum4) - (pHandleHT->wSinSum4 * pHandleHT->wCosSum6);
          
          pHandleHT->hPhaseShiftCircularMean4_6 = MCM_PhaseComputation( pHandleHT->wCosMean, pHandleHT->wSinMean);
          pHandleHT->hPhaseShiftCircularMeanDeg4_6 = pHandleHT->hPhaseShiftCircularMean4_6 * 180 / 32768;
          
          
          //cos(a - b) = cos a cos b + sin a sin b
          pHandleHT->wCosMean = (pHandleHT->wCosSum2 * pHandleHT->wCosSum6) + (pHandleHT->wSinSum2 * pHandleHT->wSinSum6);
          //sin(a - b) = sin a cos b - sin b cos a
          pHandleHT->wSinMean = (pHandleHT->wSinSum2 * pHandleHT->wCosSum6) - (pHandleHT->wSinSum6 * pHandleHT->wCosSum2);
          
          pHandleHT->hPhaseShiftCircularMean6_2 = MCM_PhaseComputation( pHandleHT->wCosMean, pHandleHT->wSinMean);
          pHandleHT->hPhaseShiftCircularMeanDeg6_2 = pHandleHT->hPhaseShiftCircularMean6_2 * 180 / 32768;
          
          
          //cos(a - b) = cos a cos b + sin a sin b
          pHandleHT->wCosMean = (pHandleHT->wCosSum3 * pHandleHT->wCosSum2) + (pHandleHT->wSinSum3 * pHandleHT->wSinSum2);
          //sin(a - b) = sin a cos b - sin b cos a
          pHandleHT->wSinMean = (pHandleHT->wSinSum3 * pHandleHT->wCosSum2) - (pHandleHT->wSinSum2 * pHandleHT->wCosSum3);
          
          pHandleHT->hPhaseShiftCircularMean2_3 = MCM_PhaseComputation( pHandleHT->wCosMean, pHandleHT->wSinMean);
          pHandleHT->hPhaseShiftCircularMeanDeg2_3 = pHandleHT->hPhaseShiftCircularMean2_3 * 180 / 32768;
          
          
          //cos(a - b) = cos a cos b + sin a sin b
          pHandleHT->wCosMean = (pHandleHT->wCosSum1 * pHandleHT->wCosSum3) + (pHandleHT->wSinSum1 * pHandleHT->wSinSum3);
          //sin(a - b) = sin a cos b - sin b cos a
          pHandleHT->wSinMean = (pHandleHT->wSinSum1 * pHandleHT->wCosSum3) - (pHandleHT->wSinSum3 * pHandleHT->wCosSum1);
          
          pHandleHT->hPhaseShiftCircularMean3_1 = MCM_PhaseComputation( pHandleHT->wCosMean, pHandleHT->wSinMean);
          pHandleHT->hPhaseShiftCircularMeanDeg3_1 = pHandleHT->hPhaseShiftCircularMean3_1 * 180 / 32768;
          
          
          //cos(a - b) = cos a cos b + sin a sin b
          pHandleHT->wCosMean = (pHandleHT->wCosSum5 * pHandleHT->wCosSum1) + (pHandleHT->wSinSum5 * pHandleHT->wSinSum1);
          //sin(a - b) = sin a cos b - sin b cos a
          pHandleHT->wSinMean = (pHandleHT->wSinSum5 * pHandleHT->wCosSum1) - (pHandleHT->wSinSum1 * pHandleHT->wCosSum5);
          
          pHandleHT->hPhaseShiftCircularMean1_5 = MCM_PhaseComputation( pHandleHT->wCosMean, pHandleHT->wSinMean);
          pHandleHT->hPhaseShiftCircularMeanDeg1_5 = pHandleHT->hPhaseShiftCircularMean1_5 * 180 / 32768;
          
          pHandleHT->sm_state = HT_RESULT;
        }
      }
      break;

      case HT_RST:
      {
        if ( pHandleHT->userWantsToRestart == true ) 
        {
          HT_Init( pHandleHT, true );
        }
        else
        {
          HT_Init( pHandleHT, false );
        }
      }
      break;
      
      case HT_RESULT:
      {
        //Waiting for the GUI to get the data: PhaseShift + Placement
        if ( pHandleHT->HT_Start == 0 )
        {
          pHandleHT->pMCI->State=STOP;
          HT_Init( pHandleHT, false );
        }
      }
      break;

      default:      
        break;
    }
}


/**
  * @brief  It is used to calculate the instantaneous difference between the
  *             the PLL and the Hall Sensors electrical angles.
  * @param  pHandleHT pointer on related component instance.
  * @retval none
  */
void HT_GetPhaseShift( HT_Handle_t * pHandleHT )
{
  MCI_State_t MCI_State = MCI_GetSTMState ( pHandleHT->pMCI );
  switch ( pHandleHT->sm_state )
  {
    case HT_GET_ANGLE_EDGE:
    {
      if ( MCI_State == RUN )
      {
        pHandleHT->hPhaseShiftInstantaneous = pHandleHT->pSTO_PLL_M1->_Super.hElAngle;
        pHandleHT->sm_state = HT_CALC_EDGE_ANGLE;
      }
    }
    default:
    break;
  }
}

/**
  * @brief  Sets the positive direction wanted by the user.
  * @param  pHandleHT: handler of HT component.
  * @param  bPP Number of motor poles pairs to be set.
  * @retval none
  */
void HT_SetMechanicalWantedDirection( HT_Handle_t * pHandleHT, uint8_t bMWD )
{
  pHandleHT->bMechanicalWantedDirection = bMWD;
}

/**
  * @brief  Sets the confirmation that the user wants to start the algo.
  * @param  pHandleHT: handler of HT component.
  * @retval none
  */
void HT_SetStart ( HT_Handle_t * pHandleHT, bool value )
{
  pHandleHT->HT_Start = value;
}

/**
  * @brief  Sets the confirmation that the user wants to restart the algo.
  * @param  pHandleHT: handler of HT component.
  * @retval none
  */
void HT_SetRestart ( HT_Handle_t * pHandleHT )
{
  pHandleHT->userWantsToRestart = true;
}

/**
  * @brief  Sets the confirmation that the user wants to abort the algo.
  * @param  pHandleHT: handler of HT component.
  * @retval none
  */
void HT_SetAbort( HT_Handle_t * pHandleHT )
{
  pHandleHT->userWantsToAbort = true;
}

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
