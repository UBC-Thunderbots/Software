/**
  ******************************************************************************
  * @file    mp_hall_tuning.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.2.0
  * @date    20-Aug-2015 18:06
  * @brief   This file contains private definition of SelfComCtrl component
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MP_HALL_TUNING_H
#define __MP_HALL_TUNING_H

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "pwm_curr_fdbk.h"
#include "ramp_ext_mngr.h"
#include "bus_voltage_sensor.h"
#include "speed_pos_fdbk.h"
#include "virtual_speed_sensor.h"
#include "open_loop.h"
#include "circle_limitation.h"
#include "pid_regulator.h"
#include "revup_ctrl.h"
#include "speed_torq_ctrl.h"
#include "r_divider_bus_voltage_sensor.h"
#include "sto_pll_speed_pos_fdbk.h"
#include "mp_one_touch_tuning.h"
#include "hall_speed_pos_fdbk.h"
#include "pmsm_motor_parameters.h"
#include "ramp_ext_mngr.h"
#include "mc_math.h"

#include "mc_interface.h"

/* Hall Sensors Pins Def------------------------------------------------------*/
#define NO_COMPLEMENT 0x0
#define H1 0x1
#define H2 0x2
#define H3 0x4
#define HALL_FLAG_TIMEOUT 1000 
#define WAIT_RAMP_TIMEOUT 10000

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
/** @addtogroup SelfComCtrl
  * @{
  */
/** @defgroup SelfComCtrl_class_private_types SelfComCtrl class private types
* @{
*/


/**
* @brief  HT_State_t enum type definition, it lists all the possible HT
          machine states.
*/
typedef enum
{
  HT_IDLE,			        /* 0 */
  HT_PRIOR_CHECK,		        /* 1 */
  HT_START_RAMP,		        /* 2 */
  HT_WAIT_RAMP,			        /* 3 */
  HT_WAIT_HALL_FLAG,		        /* 4 */
  HT_CHECK_HALL_RELIABILITY,	        /* 5 */
  HT_WAIT_USER_DIRECTION_CHOICE,        /* 6 */
  HT_ERROR_RELIABILITY,			/* 7 */
  HT_ERROR_PINS_READING,		/* 8 */
  HT_WARNING_PHASES_NEED_SWAP,		/* 9 */
  HT_DETECTING_CONF,			/* 10 */
  HT_DETERMINE_PTC,			/* 11 */
  HT_DETECTING_SWAP,			/* 12 */
  HT_WAIT_STABILIZATION, 		/* 13 */
  HT_DETECTING_EDGE,
  HT_GET_ANGLE_EDGE,
  HT_CALC_EDGE_ANGLE,
  HT_EDGE_COMPUTATION,
  HT_RST,				/* 16 */
  HT_RESULT				/* 17 */
} HT_State_t;


/**
* @brief  HT_State_t enum type definition, it lists all the possible HT_SHIF_EDGE
          machine states.
*/
typedef enum
{
  SHIFT_EDGE_IDLE,	        /* 0 */
  SHIFT_EDGE_1,	                /* 1 */
  SHIFT_EDGE_2,	                /* 2 */
  SHIFT_EDGE_3,	                /* 3 */
  SHIFT_EDGE_4,	                /* 4 */
  SHIFT_EDGE_5,	                /* 5 */
  SHIFT_EDGE_6,                 /* 6 */
  SHIFT_EDGE_END		/* 7 */
} ShiftEdge_State_t;


/**
  * * @brief  Handle structure of the HallTuning.
  */
typedef struct
{
      MCI_Handle_t  *pMCI;           /*!< State machine of related MC.*/
      OTT_Handle_t *pOTT;
      HALL_Handle_t *pHALL_M1;
      STO_PLL_Handle_t *pSTO_PLL_M1;
      
  HT_State_t sm_state; /*!< HT state machine state.*/
  
  bool HT_Start;
  bool directionAlreadyChosen;
  bool userWantsToRestart;
  bool userWantsToAbort;
  bool flagState0; /*!< true if current HALL state configuration is H1=H2=H3=0. Used to detect 60 degrees Hall configuration */
  bool flagState1; /*!< true if current HALL state configuration H1=H2=H3=1. Used to detect 60 degrees Hall configuration */
  bool H1Connected;
  bool H2Connected;
  bool H3Connected;
  bool PTCWellPositioned; 
  bool waitHallFlagCompleted;
  bool reliable;
  bool edgeAngleDirPos;
  
  uint8_t bPlacement;
  uint8_t bMechanicalWantedDirection;
  uint8_t bNewH1;
  uint8_t bNewH2;
  uint8_t bNewH3;
  uint8_t bProgressPercentage;
  
  int8_t  bPinToComplement;
  
  uint16_t hHallFlagCnt;
  uint16_t hWaitRampCnt;
  uint16_t hShiftAngleDepth;
  
  int16_t hPhaseShiftInstantaneous;
  int16_t hPhaseShiftCircularMean;
  int16_t hPhaseShiftCircularMeanDeg;
  int16_t hPhaseShiftCircularMeanNeg;
  int16_t hPhaseShiftCircularMeanNegDeg;
  
  uint32_t previousH1;
  uint32_t previousH2;
  uint32_t previousH3;
  
  int32_t wSinMean;
  int32_t wCosMean;
  
  ShiftEdge_State_t shiftEdge_state;
  int32_t wSinSum1;
  int32_t wCosSum1;
  int32_t wSinSum2;
  int32_t wCosSum2;
  int32_t wSinSum3;
  int32_t wCosSum3;
  int32_t wSinSum4;
  int32_t wCosSum4;
  int32_t wSinSum5;
  int32_t wCosSum5;
  int32_t wSinSum6;
  int32_t wCosSum6;
  
  int16_t hPhaseShiftCircularMean5_1;
  int16_t hPhaseShiftCircularMeanDeg5_1;
  int16_t hPhaseShiftCircularMean1_3;
  int16_t hPhaseShiftCircularMeanDeg1_3;
  int16_t hPhaseShiftCircularMean3_2;
  int16_t hPhaseShiftCircularMeanDeg3_2;
  int16_t hPhaseShiftCircularMean2_6;
  int16_t hPhaseShiftCircularMeanDeg2_6;
  int16_t hPhaseShiftCircularMean6_4;
  int16_t hPhaseShiftCircularMeanDeg6_4;
  int16_t hPhaseShiftCircularMean4_5;
  int16_t hPhaseShiftCircularMeanDeg4_5;
  
  
  int16_t hPhaseShiftCircularMean5_4;
  int16_t hPhaseShiftCircularMeanDeg5_4;
  int16_t hPhaseShiftCircularMean4_6;
  int16_t hPhaseShiftCircularMeanDeg4_6;
  int16_t hPhaseShiftCircularMean6_2;
  int16_t hPhaseShiftCircularMeanDeg6_2;
  int16_t hPhaseShiftCircularMean2_3;
  int16_t hPhaseShiftCircularMeanDeg2_3;
  int16_t hPhaseShiftCircularMean3_1;
  int16_t hPhaseShiftCircularMeanDeg3_1;
  int16_t hPhaseShiftCircularMean1_5;
  int16_t hPhaseShiftCircularMeanDeg1_5;

} HT_Handle_t;




void HT_Init( HT_Handle_t * pHandleHT, bool pRST );
void HT_MF( HT_Handle_t * pHandleHT );
void HT_GetPhaseShift( HT_Handle_t * pHandleHT );
void HT_Stop( HT_Handle_t * pHandleHT );
void HT_SetMechanicalWantedDirection( HT_Handle_t * pHandleHT, uint8_t bMWD );
void HT_SetStart ( HT_Handle_t * pHandleHT, bool value );
void HT_SetRestart ( HT_Handle_t * pHandleHT );
void HT_SetAbort( HT_Handle_t * pHandleHT );

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /*__MP_HALL_TUNING_H*/

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/