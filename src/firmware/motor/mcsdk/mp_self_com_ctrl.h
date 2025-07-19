/**
  ******************************************************************************
  * @file    mp_self_com_ctrl.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.2.0
  * @date    20-Aug-2015 18:06
  * @brief   This file contains private definition of SelfComCtrl component
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License")
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
#ifndef MP_SELF_COM_CTRL_H
#define MP_SELF_COM_CTRL_H

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
#include "mp_hall_tuning.h"

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup SelfComCtrl
  * @{
  */

#define RSCURRLEVELNUM 4u
#define EMF_BUFF_VAL 5u
#define CMD_SC_STOP 0u
#define CMD_SC_START 1u
#define CMD_HT_START 2u
#define CMD_HT_RESTART 3u
#define CMD_HT_ABORT 4u
#define CMD_HT_END 5u
#define CMD_PPD_START 6u

/** @defgroup SelfComCtrl_class_private_types SelfComCtrl class private types
  * @{
  */

/**
  * @brief  LS detection states
  */
typedef enum
{
  LSDET_DECAY,
  LSDET_HOLD,
  LSDET_RISE
} LSDetState_t;

/**
  * @brief  KE detection states
  */
typedef enum
{
  KEDET_REVUP,
  KEDET_DETECTION,
  KEDET_SET_OBS_PARAMS,
  KEDET_STABILIZEPLL,
  KEDET_RUN,
  KEDET_RESTART
} KEDetState_t;

/**
  * @brief  SCC_State_t enum type definition, it lists all the possible SCC state
          machine states.
  */
typedef enum
{
  SCC_IDLE,
  SCC_DUTY_DETECTING_PHASE,
  SCC_ALIGN_PHASE,
  SCC_RS_DETECTING_PHASE_RAMP,
  SCC_RS_DETECTING_PHASE,
  SCC_LS_DETECTING_PHASE,
  SCC_WAIT_RESTART,
  SCC_RESTART_SCC,
  SCC_KE_DETECTING_PHASE,
  SCC_PHASE_STOP,
  SCC_CALIBRATION_END,
  SCC_PP_DETECTION_RAMP,
  SCC_PP_DETECTION_PHASE_RAMP,
  SCC_PP_DETECTION_PHASE
} SCC_State_t;

/**
  * @brief  KE detection speed ramp status
  */
typedef enum
{
  RampIdle,    /* Ramp not started yet */
  RampOngoing, /* Ramp is ongoing */
  RampSucces,  /* The motor has been accelerated up to the target speed */
  MotorStill,  /* Motor didn't move at all */
  LoseControl, /* Motor start to follow acceleration but didn't reach the target speed */
} AccResult_t;


/**
  * @brief  SelfComCtrl parameters definition
  */
typedef const struct
{
  RampExtMngr_Handle_t rampExtMngrParams; /*!< Ramp manager used by SCC.*/
  float fRshunt;                         /*!< Value of shunt resistor.*/
  float fAmplificationGain;              /*!< Current sensing amplification gain.*/
  float fVbusConvFactor;                 /*!< Bus voltage conversion factor.*/
  float fVbusPartitioningFactor;         /*!< Bus voltage partitioning factor.
                                              (Vmcu / Bus voltage conversion factor).*/
  float fRVNK;                           /*!< Power stage calibration factor.
                                              (Measured experimentally).*/
  float fRSMeasCurrLevelMax;             /*!< Maximum level of DC current used
                                              for RS measurement.*/
  uint16_t hDutyRampDuration;            /*!< Duration of voltage ramp executed
                                              for the duty cycle determination
                                              stage.*/
  uint16_t hAlignmentDuration;           /*!< Duration of the alignment stage.*/
  uint16_t hRSDetectionDuration;         /*!< Duration of R detection stage.*/
  float fLdLqRatio;                      /*!< Ld vs Lq ratio.*/
  float fCurrentBW;                      /*!< Bandwidth of current regulator.*/
  bool bPBCharacterization;              /*!< Set to true for characterization
                                              of power board, otherwise false.*/
  int32_t wNominalSpeed;                 /*!< Nominal speed set by the user expressed in RPM.*/
  uint16_t hPWMFreqHz;                   /*!< PWM frequency used for the test.*/
  uint8_t bFOCRepRate;                   /*!< FOC repetition rate used for the test.*/
  float fMCUPowerSupply;                 /*!< MCU Power Supply */
  float IThreshold;
} SCC_Params_t, *pSCC_Params_t;

/**
  * * @brief  Handle structure of the SelfComCtrl.
  */
typedef struct
{
  PWMC_Handle_t *pPWMC;          /*!< Current feedback and PWM object used.*/
  RDivider_Handle_t *pVBS;           /*!< Bus voltage sensor used.*/
  pFOCVars_t pFOCVars;  /*!< Related structure of FOC vars.*/
  MCI_Handle_t  *pMCI;           /*!< State machine of related MC.*/
  VirtualSpeedSensor_Handle_t *pVSS;            /*!< VSS used.*/
  CircleLimitation_Handle_t *pCLM;            /*!< Circle limitation used.*/
  PID_Handle_t *pPIDIq;           /*!< Iq PID used.*/
  PID_Handle_t *pPIDId;           /*!< Id PID used.*/
  RevUpCtrl_Handle_t *pRevupCtrl; /*!< RUC used.*/
  STO_PLL_Handle_t *pSTO;            /*!< State Observer used.*/
  SpeednTorqCtrl_Handle_t *pSTC;            /*!< Speed and torque controller used.*/
  OTT_Handle_t *pOTT;
  HT_Handle_t *pHT;

  SCC_State_t sm_state; /*!< SCC state machine state.*/
  RampExtMngr_Handle_t *pREMng;        /*!< Ramp manager used.*/
  uint16_t hDutyMax;    /*!< Duty cycle to be applied to reach the target current.*/
  LSDetState_t LSDetState;/*!< Internal state during LS detection. */
  KEDetState_t KEDetState;/*!< Internal state during KE detection. */

  float fTPWM;            /*!< PWM period in second. */
  float fFocRate;         /*!< FOC execution rate. */
  float fPP;              /*!< Motor poles pairs. */
  int16_t hMax_voltage;   /*!< Maximum readable voltage. */
  float fMax_current;     /*!< Maximum readable current. */
  float fTargetCurr;      /*!< Instantaneous value of target current used for R
                               estimation.*/
  float fLastTargetCurr;  /*!< Last value of set nominal current used for R
                               estimation.*/
  float fRSCurrLevelStep; /*!< Step of current used for R estimation.*/
  float fBusV;            /*!< Stores the last Vbus measurement.*/
  float fRS;              /*!< Stores the last R estimation.*/
  float fLS;              /*!< Stores the last L estimation.*/
  float fKe;              /*!< Stores the last Ke estimation.*/
  float fImaxArray[RSCURRLEVELNUM]; /*!< Array to store current measurement
                                         values for R estimation.*/
  float fVmaxArray[RSCURRLEVELNUM]; /*!< Array to store voltage measurement
                                         values for R estimation.*/
  uint8_t bRSCurrLevelTests;        /*!< Counter to store I and V values for R
                                         estimation.*/
  float fImax;            /*!< Stores the last current measurement done, it
                               will be used to compute the 63% for electrical
                               time constant measurement.*/
  float fItau;            /*!< Stores the last computed electrical time constant.*/
  uint8_t  bDutyDetTest;  /*!< Number of test done in duty determination phase.*/
  uint32_t wLSTimeCnt;    /*!< Time counter for LS determination.*/
  uint32_t wLSTestCnt;    /*!< Counter for LS tests. */
  float fLSsum;           /*!< Sum of estimated LS for mean.*/
  float fIsum;            /*!< Sum of measured current for mean.*/
  float fVsum;            /*!< Sum of estimated voltage for mean.*/
  uint32_t wICnt;         /*!< Counter of I and V acquired.*/
  uint32_t index;         /*!< Counter of I and V acquired.*/
  float fIqsum;           /*!< Sum of measured Iq for mean.*/
  float fVqsum;           /*!< Sum of measured Vq for mean.*/
  float fVdsum;           /*!< Sum of measured Vd for mean.*/
  float fFesum;           /*!< Sum of electrical frequency for mean.*/
  uint32_t wKeAcqCnt;     /*!< Counter of Iq, Vq and Vd acquired.*/
  float fEstNominalSpdRPM;/*!< Estimated nominal speed.*/
  float k1, k2;           /*!< Coefficient computed for state observer tuning.*/
  int8_t stabCnt;         /*!< Stabilization counter.*/
  float fLdLqRatio;       /*!< Ld vs Lq ratio.*/
  int32_t wNominalSpeed;  /*!< Nominal speed set by the user expressed in RPM.*/
  int32_t wMaxOLSpeed;    /*!< Maximum speed that can be sustained in the startup.*/
  uint32_t wAccRPMs;      /*!< Acceleration ramp for the motor expressed in
                               RMP/s.*/
  bool accRampLock;       /*!< It become true if the motor follow the acceleration. */
  float fEm_val[EMF_BUFF_VAL]; /*!< Buffer used for linear regression (BEMF amplitude).*/
  float fw_val[EMF_BUFF_VAL]; /*!< Buffer used for linear regression (Angular velocity).*/
  uint16_t hVal_ctn;       /*!< Counter for the buffer used for linear regression.*/
  bool startComputation;  /*!< It becomes true if the buffer used for linear
                               regression has been filled.*/
  uint16_t hTimeOutCnt;    /*!< Time out counter to assert the motor still
                               condition during acceleration ramp.*/
  uint32_t wLoseControlAtRPM; /*!< Last speed forced before loosing control during
                                 acceleration ramp.*/
  AccResult_t res;        /*!< Result state of the last acceleration ramp.*/
  float fLastValidI;      /*!< Last valid current measurement during SCC_DUTY_DETECTING_PHASE */
  uint16_t hMFCount;      /*!< Counter of MF to be wait after OC or loose control.*/
  uint16_t hMFTimeout;      /*!< Counter of MF to be wait after OC or loose control.*/
  float fCurrentBW;       /*!< Bandwidth of speed regulator.*/
  uint8_t bMPOngoing;     /*!< It is 1 if MP is ongoing, 0 otherwise.*/
  uint32_t wSpeedThToValidateStartupRPM; /*!< Speed threshold to validate the startup.*/
  float IaBuff[256];
  bool detectBemfState;
  
  bool polePairDetection; //profiler ppDetection
  uint32_t ppDtcCnt;           /*!< Counter of pole pairs detection time.*/

  pSCC_Params_t pSCC_Params_str;  /**< SelfComCtrl parameters */

} SCC_Handle_t;


/**
  * @brief  Initializes all the object variables, usually it has to be called
  *         once right after object creation.
  * @param  this related object of class CSCC.
  * @retval none.
  */
void SCC_Init(SCC_Handle_t *pHandle);
/**
  * @brief  It should be called before each motor restart.
  * @param this related object of class CSCC.
  * @retval bool It return false if function fails to start the SCC.
  */
bool SCC_Start(SCC_Handle_t *pHandle);

/**
  * @brief  It should be called before each motor stop.
  * @param this related object of class CSCC.
  * @retval none.
  */
void SCC_Stop(SCC_Handle_t *pHandle);

/**
  * @brief  It feed the required phase voltage to the inverter.
  * @param  this related object of class CSCC.
  * @retval It returns the code error 'MC_DURATION' if any, 'MC_NO_ERROR'
  *         otherwise. These error codes are defined in mc_type.h
  */
uint16_t SCC_SetPhaseVoltage(SCC_Handle_t *pHandle);

/**
  * @brief  Medium frequency task.
  * @param  this related object of class CSCC.
  * @retval none
  */
void SCC_MF(SCC_Handle_t *pHandle);

/**
  * @brief  Check overcurrent during RL detetction and restart the procedure
  *         with less current.
  * @param  this related object of class CSCC.
  * @retval none.
  */
void SCC_CheckOC_RL(SCC_Handle_t *pHandle);

uint8_t SCC_CMD(SCC_Handle_t *pHandle, uint16_t rxLength, uint8_t *rxBuffer, int16_t txSyncFreeSpace, uint16_t *txLength, uint8_t *txBuffer);
void SCC_SetNominalCurrent(SCC_Handle_t *pHandle, float fCurrent);
void SCC_SetLdLqRatio(SCC_Handle_t *pHandle, float fLdLqRatio);
void SCC_SetNominalSpeed(SCC_Handle_t *pHandle, int32_t wNominalSpeed);
void SCC_SetPolesPairs(SCC_Handle_t *pHandle, uint8_t bPP);
void SCC_SetCurrentBandwidth(SCC_Handle_t *pHandle, float fCurrentBW);
uint8_t SCC_GetState(SCC_Handle_t *pHandle);
uint8_t SCC_GetSteps(SCC_Handle_t *pHandle);
uint8_t SCC_GetFOCRepRate(SCC_Handle_t *pHandle);
uint16_t SCC_GetPWMFrequencyHz(SCC_Handle_t *pHandle);
uint32_t SCC_GetRs(SCC_Handle_t *pHandle);
uint32_t SCC_GetLs(SCC_Handle_t *pHandle);
uint32_t SCC_GetKe(SCC_Handle_t *pHandle);
uint32_t SCC_GetVbus(SCC_Handle_t *pHandle);
float SCC_GetNominalCurrent(SCC_Handle_t *pHandle);
float SCC_GetLdLqRatio(SCC_Handle_t *pHandle);
int32_t SCC_GetNominalSpeed(SCC_Handle_t *pHandle);
float SCC_GetCurrentBandwidth(SCC_Handle_t *pHandle);
float SCC_GetStartupCurrentAmp(SCC_Handle_t *pHandle);
int32_t SCC_GetEstMaxAcceleration(SCC_Handle_t *pHandle);
int32_t SCC_GetEstMaxOLSpeed(SCC_Handle_t *pHandle);

uint16_t SCC_GetUnderVoltageThreshold(SCC_Handle_t *pHandle);
uint16_t SCC_GetOverVoltageThreshold(SCC_Handle_t *pHandle);
void SCC_SetOverVoltageThreshold(SCC_Handle_t *pHandle,uint16_t value);
void SCC_SetUnderVoltageThreshold(SCC_Handle_t *pHandle,uint16_t value);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /*MP_SELF_COM_CTRL_H*/

/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
