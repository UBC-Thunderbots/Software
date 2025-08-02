
/**
  ******************************************************************************
  * @file    mc_config.c
  * @author  Motor Control SDK Team,ST Microelectronics
  * @brief   Motor Control Subsystem components configuration and handler structures.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044,the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
//cstat -MISRAC2012-Rule-21.1
#include "firmware/motor/main.h" //cstat !MISRAC2012-Rule-21.1
//cstat +MISRAC2012-Rule-21.1
#include "firmware/motor/mc_config.h"
#include "firmware/motor/mc_parameters.h"
#include "firmware/motor/mcsdk/mc_type.h"
#include "firmware/motor/mcsdk/pqd_motor_power_measurement.h"
#include "firmware/motor/parameters_conversion.h"

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

#define FREQ_RATIO 1                /* Dummy value for single drive */
#define FREQ_RELATION HIGHEST_FREQ  /* Dummy value for single drive */

/* USER CODE BEGIN Additional define */

/* USER CODE END Additional define */

PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM1 =
{
  .ConvFact = PQD_CONVERSION_FACTOR
};

/**
  * @brief  PI / PID Speed loop parameters Motor 1.
  */
PID_Handle_t PIDSpeedHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_SPEED_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_SPEED_KI_DEFAULT,
  .wUpperIntegralLimit = (int32_t)(IQMAX * SP_KIDIV),
  .wLowerIntegralLimit = -(int32_t)(IQMAX * SP_KIDIV),
  .hUpperOutputLimit   = (int16_t)IQMAX,
  .hLowerOutputLimit   = -(int16_t)IQMAX,
  .hKpDivisor          = (uint16_t)SP_KPDIV,
  .hKiDivisor          = (uint16_t)SP_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)SP_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)SP_KIDIV_LOG,
  .hDefKdGain          = 0x0000U,
  .hKdDivisor          = 0x0000U,
  .hKdDivisorPOW2      = 0x0000U,
};

/**
  * @brief  PI / PID Iq loop parameters Motor 1.
  */
PID_Handle_t PIDIqHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_TORQUE_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_TORQUE_KI_DEFAULT,
  .wUpperIntegralLimit = (int32_t)(INT16_MAX * TF_KIDIV),
  .wLowerIntegralLimit = (int32_t)(-INT16_MAX * TF_KIDIV),
  .hUpperOutputLimit   = INT16_MAX,
  .hLowerOutputLimit   = -INT16_MAX,
  .hKpDivisor          = (uint16_t)TF_KPDIV,
  .hKiDivisor          = (uint16_t)TF_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG,
  .hDefKdGain          = 0x0000U,
  .hKdDivisor          = 0x0000U,
  .hKdDivisorPOW2      = 0x0000U,
};

/**
  * @brief  PI / PID Id loop parameters Motor 1.
  */
PID_Handle_t PIDIdHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_FLUX_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_FLUX_KI_DEFAULT,
  .wUpperIntegralLimit = (int32_t)(INT16_MAX * TF_KIDIV),
  .wLowerIntegralLimit = (int32_t)(-INT16_MAX * TF_KIDIV),
  .hUpperOutputLimit   = INT16_MAX,
  .hLowerOutputLimit   = -INT16_MAX,
  .hKpDivisor          = (uint16_t)TF_KPDIV,
  .hKiDivisor          = (uint16_t)TF_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG,
  .hDefKdGain          = 0x0000U,
  .hKdDivisor          = 0x0000U,
  .hKdDivisorPOW2      = 0x0000U,
};

/**
  * @brief  SpeednTorque Controller parameters Motor 1.
  */
SpeednTorqCtrl_Handle_t SpeednTorqCtrlM1 =
{
  .STCFrequencyHz             = MEDIUM_FREQUENCY_TASK_RATE,
  .MaxAppPositiveMecSpeedUnit = (uint16_t)(MAX_APPLICATION_SPEED_UNIT),
  .MinAppPositiveMecSpeedUnit = (uint16_t)(MIN_APPLICATION_SPEED_UNIT),
  .MaxAppNegativeMecSpeedUnit = (int16_t)(-MIN_APPLICATION_SPEED_UNIT),
  .MinAppNegativeMecSpeedUnit = (int16_t)(-MAX_APPLICATION_SPEED_UNIT),
  .MaxPositiveTorque          = (int16_t)NOMINAL_CURRENT,
  .MinNegativeTorque          = -(int16_t)NOMINAL_CURRENT,
  .ModeDefault                = DEFAULT_CONTROL_MODE,
  .MecSpeedRefUnitDefault     = (int16_t)(DEFAULT_TARGET_SPEED_UNIT),
  .TorqueRefDefault           = (int16_t)DEFAULT_TORQUE_COMPONENT,
  .IdrefDefault               = (int16_t)DEFAULT_FLUX_COMPONENT,
};

PWMC_R1_Handle_t PWM_Handle_M1 =
{
  {
    .pFctGetPhaseCurrents       = &R1_GetPhaseCurrents,
    .pFctSetOffsetCalib         = &R1_SetOffsetCalib,
    .pFctGetOffsetCalib         = &R1_GetOffsetCalib,
    .pFctSwitchOffPwm           = &R1_SwitchOffPWM,
    .pFctSwitchOnPwm            = &R1_SwitchOnPWM,
    .pFctCurrReadingCalib       = &R1_CurrentReadingCalibration,
    .pFctTurnOnLowSides         = &R1_TurnOnLowSides,
    .pFctSetADCSampPointSectX   = &R1_CalcDutyCycles,
    .pFctOCPSetReferenceVoltage = MC_NULL,
    .pFctRLDetectionModeEnable  = MC_NULL,
    .pFctRLDetectionModeDisable = MC_NULL,
    .pFctRLDetectionModeSetDuty = MC_NULL,
    .pFctRLTurnOnLowSidesAndStart = MC_NULL,
    .LowSideOutputs    = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,
    .pwm_en_u_port     = MC_NULL,
    .pwm_en_u_pin      = (uint16_t)0,
    .pwm_en_v_port     = MC_NULL,
    .pwm_en_v_pin      = (uint16_t)0,
    .pwm_en_w_port     = MC_NULL,
    .pwm_en_w_pin      = (uint16_t)0,
    .hT_Sqrt3                   = (PWM_PERIOD_CYCLES*SQRT3FACTOR) / 16384u,
    .Sector                     = 0,
    .CntPhA                     = 0,
    .CntPhB                     = 0,
    .CntPhC                     = 0,
    .SWerror                    = 0,
    .TurnOnLowSidesAction       = false,
    .OffCalibrWaitTimeCounter   = 0,
    .Motor                      = 0,
    .RLDetectionMode            = false,
    .SingleShuntTopology        = true,
    .Ia                         = 0,
    .Ib                         = 0,
    .Ic                         = 0,
    .LPFIqd_const               = LPF_FILT_CONST,
    .DTCompCnt                  = DTCOMPCNT,
    .PWMperiod                  = PWM_PERIOD_CYCLES,
    .Ton                        = TON,
    .Toff                       = TOFF,
    .OverCurrentFlag            = false,
    .OverVoltageFlag            = false,
    .BrakeActionLock            = false,
    .driverProtectionFlag       = false,
  },

  .DmaBuffCCR = {0,0,0,0,0,0},
  .PhaseOffset                  = 0,
  .Half_PWMPeriod               = PWM_PERIOD_CYCLES / 2u,
  .CntSmp1                      = 0,
  .CntSmp2                      = 0,
  .sampCur1                     = 0,
  .sampCur2                     = 0,
  .CurrAOld                     = 0,
  .CurrBOld                     = 0,
  .Index                        = 0,
  .iflag                        = 0,
  .TCCnt                        = 0U,
  .UpdateFlagBuffer             = 0,
  .TCDoneFlag                   = true,
  .pParams_str                  = &R1_ParamsM1,
};

OpenLoop_Handle_t OpenLoop_ParamsM1 =
{
  .hDefaultVoltage = OPEN_LOOP_VOLTAGE_d,
  .VFMode          = OPEN_LOOP_VF,
  .hVFOffset       = OPEN_LOOP_OFF,
  .hVFSlope        = OPEN_LOOP_K
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - HALL.
  */
HALL_Handle_t HALL_M1 =
{
  ._Super =
  {
    .bElToMecRatio             = POLE_PAIR_NUM,
    .hMaxReliableMecSpeedUnit  = (uint16_t)(1.15 * MAX_APPLICATION_SPEED_UNIT),
    .hMinReliableMecSpeedUnit  = (uint16_t)(MIN_APPLICATION_SPEED_UNIT),
    .bMaximumSpeedErrorsNumber = M1_SS_MEAS_ERRORS_BEFORE_FAULTS,
    .hMaxReliableMecAccelUnitP = 65535,
    .hMeasurementFrequency     = TF_REGULATION_RATE_SCALED,
    .DPPConvFactor             = DPP_CONV_FACTOR,
  },

  .SensorPlacement             = HALL_SENSORS_PLACEMENT,
  .PhaseShift                  = (int16_t)(HALL_PHASE_SHIFT * 65536 / 360),
  .SpeedSamplingFreqHz         = MEDIUM_FREQUENCY_TASK_RATE,
  .SpeedBufferSize             = HALL_AVERAGING_FIFO_DEPTH,
  .TIMClockFreq                = HALL_TIM_CLK,
  .TIMx                        = TIM2,
  .ICx_Filter                  = M1_HALL_IC_FILTER_LL,
  .PWMFreqScaling              = PWM_FREQ_SCALING,
  .HallMtpa                    = HALL_MTPA,
  .H1Port                      = M1_HALL_H1_GPIO_Port,
  .H1Pin                       = M1_HALL_H1_Pin,
  .H2Port                      = M1_HALL_H2_GPIO_Port,
  .H2Pin                       = M1_HALL_H2_Pin,
  .H3Port                      = M1_HALL_H3_GPIO_Port,
  .H3Pin                       = M1_HALL_H3_Pin,
};

/** RAMP for Motor1
  *
  */
RampExtMngr_Handle_t RampExtMngrHFParamsM1 =
{
  .FrequencyHz = TF_REGULATION_RATE
};

/**
  * @brief  CircleLimitation Component parameters Motor 1 - Base Component.
  */
CircleLimitation_Handle_t CircleLimitationM1 =
{
  .MaxModule = MAX_MODULE,
  .MaxVd     = (uint16_t)((MAX_MODULE * 950) / 1000),
};

FOCVars_t FOCVars[NBR_OF_MOTORS];
RampExtMngr_Handle_t *pREMNG[NBR_OF_MOTORS];
SpeednTorqCtrl_Handle_t *pSTC[NBR_OF_MOTORS]    = {&SpeednTorqCtrlM1};
NTC_Handle_t *pTemperatureSensor[NBR_OF_MOTORS] = {&TempSensor_M1};
PID_Handle_t *pPIDIq[NBR_OF_MOTORS]             = {&PIDIqHandle_M1};
PID_Handle_t *pPIDId[NBR_OF_MOTORS]             = {&PIDIdHandle_M1};
PQD_MotorPowMeas_Handle_t *pMPM[NBR_OF_MOTORS]  = {&PQD_MotorPowMeasM1};

MCI_Handle_t Mci[NBR_OF_MOTORS] =
{
  {
    .pSTC = &SpeednTorqCtrlM1,
    .pFOCVars = &FOCVars[0],
    .pVSS = &VirtualSpeedSensorM1,
    .pPWM = &PWM_Handle_M1._Super,
    .lastCommand = MCI_NOCOMMANDSYET,
    .hFinalSpeed = 0,
    .hFinalTorque = 0,
    .pScale = &scaleParams_M1,
    .hDurationms = 0,
    .DirectCommand = MCI_NO_COMMAND,
    .State = IDLE,
    .CurrentFaults = MC_NO_FAULTS,
    .PastFaults = MC_NO_FAULTS,
    .CommandState = MCI_BUFFER_EMPTY,
  },

};

/* USER CODE BEGIN Additional configuration */

/* USER CODE END Additional configuration */

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/

