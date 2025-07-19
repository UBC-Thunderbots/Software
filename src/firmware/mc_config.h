/**
  ******************************************************************************
  * @file    mc_config.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Motor Control Subsystem components configuration and handler
  *          structures declarations.
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
  */

#ifndef MC_CONFIG_H
#define MC_CONFIG_H

#include "firmware/mc_interface.h"
#include "firmware/mcsdk/pid_regulator.h"
#include "firmware/mcsdk/speed_torq_ctrl.h"
#include "firmware/mcsdk/virtual_speed_sensor.h"
#include "firmware/mcsdk/ntc_temperature_sensor.h"
#include "firmware/mcsdk/revup_ctrl.h"
#include "firmware/mcsdk/pwm_curr_fdbk.h"
#include "firmware/mcsdk/mc_configuration_registers.h"
#include "firmware/mcsdk/r_divider_bus_voltage_sensor.h"
#include "firmware/mcsdk/virtual_bus_voltage_sensor.h"
#include "firmware/mcsdk/pqd_motor_power_measurement.h"

#include "firmware/r1_ps_pwm_curr_fdbk.h"

#include "firmware/encoder_speed_pos_fdbk.h"
#include "firmware/enc_align_ctrl.h"

#include "firmware/ramp_ext_mngr.h"
#include "firmware/circle_limitation.h"

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

extern PID_Handle_t PIDSpeedHandle_M1;
extern PID_Handle_t PIDIqHandle_M1;
extern PID_Handle_t PIDIdHandle_M1;
extern NTC_Handle_t TempSensor_M1;

extern PWMC_R1_Handle_t PWM_Handle_M1;

extern SpeednTorqCtrl_Handle_t SpeednTorqCtrlM1;
extern PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM1;
extern PQD_MotorPowMeas_Handle_t *pPQD_MotorPowMeasM1;
extern VirtualSpeedSensor_Handle_t VirtualSpeedSensorM1;
extern ENCODER_Handle_t ENCODER_M1;
extern EncAlign_Handle_t EncAlignCtrlM1;
extern RDivider_Handle_t BusVoltageSensor_M1;
extern CircleLimitation_Handle_t CircleLimitationM1;
extern RampExtMngr_Handle_t RampExtMngrHFParamsM1;

extern MCI_Handle_t Mci[NBR_OF_MOTORS];
extern SpeednTorqCtrl_Handle_t *pSTC[NBR_OF_MOTORS];
extern PID_Handle_t *pPIDIq[NBR_OF_MOTORS];
extern PID_Handle_t *pPIDId[NBR_OF_MOTORS];
extern NTC_Handle_t *pTemperatureSensor[NBR_OF_MOTORS];
extern PQD_MotorPowMeas_Handle_t *pMPM[NBR_OF_MOTORS];
extern MCI_Handle_t* pMCI[NBR_OF_MOTORS];
/* USER CODE BEGIN Additional extern */

/* USER CODE END Additional extern */

#endif /* MC_CONFIG_H */
/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
