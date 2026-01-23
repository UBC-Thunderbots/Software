
/**
 ******************************************************************************
 * @file    mc_config.h
 * @author  Motor Control SDK Team, ST Microelectronics
 * @brief   Motor Control Subsystem components configuration and handler
 *          structures declarations.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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

#include "firmware/motor/mc_config_common.h"
#include "firmware/motor/mcsdk/circle_limitation.h"
#include "firmware/motor/mcsdk/hall_speed_pos_fdbk.h"
#include "firmware/motor/mcsdk/open_loop.h"
#include "firmware/motor/mcsdk/pid_regulator.h"
#include "firmware/motor/mcsdk/pqd_motor_power_measurement.h"
#include "firmware/motor/mcsdk/pwm_curr_fdbk.h"
#include "firmware/motor/mcsdk/ramp_ext_mngr.h"
#include "firmware/motor/mcsdk/revup_ctrl.h"
#include "firmware/motor/mcsdk/speed_torq_ctrl.h"
#include "firmware/motor/r1_ps_pwm_curr_fdbk.h"

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

extern PID_Handle_t PIDIqHandle_M1;
extern PID_Handle_t PIDIdHandle_M1;
extern PWMC_R1_Handle_t PWM_Handle_M1;
extern PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM1;
extern PQD_MotorPowMeas_Handle_t *pPQD_MotorPowMeasM1;

extern CircleLimitation_Handle_t CircleLimitationM1;
extern RampExtMngr_Handle_t RampExtMngrHFParamsM1;
extern OpenLoop_Handle_t OpenLoop_ParamsM1;
extern RampExtMngr_Handle_t *pREMNG[NBR_OF_MOTORS];
extern FOCVars_t FOCVars[NBR_OF_MOTORS];
extern PID_Handle_t *pPIDIq[NBR_OF_MOTORS];
extern PID_Handle_t *pPIDId[NBR_OF_MOTORS];
extern PQD_MotorPowMeas_Handle_t *pMPM[NBR_OF_MOTORS];
extern MCI_Handle_t *pMCI[NBR_OF_MOTORS];
extern SpeednTorqCtrl_Handle_t *pSTC[NBR_OF_MOTORS];
extern MCI_Handle_t Mci[NBR_OF_MOTORS];
extern SpeednTorqCtrl_Handle_t SpeednTorqCtrlM1;
extern PID_Handle_t PIDSpeedHandle_M1;
extern HALL_Handle_t HALL_M1;

/* USER CODE BEGIN Additional extern */

/* USER CODE END Additional extern */

#endif /* MC_CONFIG_H */
/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
