/**
  ******************************************************************************
  * @file    trajectory_ctrl.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides all definitions and functions prototypes for the
  *          the Position Control component of the Motor Control SDK.
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
  * @ingroup PositionControl
  */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef TRAJCTRL_H
#define TRAJCTRL_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "speed_torq_ctrl.h"
#include "enc_align_ctrl.h"

#define RADTOS16 10430.378350470452725f            /* 2^15/Pi */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define Z_ALIGNMENT_DURATION  2                     /* 2 seconds */
#define Z_ALIGNMENT_NB_ROTATION (2.0f * M_PI)       /* 1 turn in 2 seconds allowed to find the "Z" signal  */

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup PositionControl
  * @{
  */

typedef enum
{
  TC_READY_FOR_COMMAND  = 0,
  TC_MOVEMENT_ON_GOING = 1,
  TC_TARGET_POSITION_REACHED = 2,
  TC_FOLLOWING_ON_GOING = 3
} PosCtrlStatus_t;

typedef enum
{
  TC_AWAITING_FOR_ALIGNMENT = 0,
  TC_ZERO_ALIGNMENT_START = 1,
  TC_ALIGNMENT_COMPLETED = 2,
  TC_ABSOLUTE_ALIGNMENT_NOT_SUPPORTED = 3,   /* Encoder sensor without "Z" output signal */
  TC_ABSOLUTE_ALIGNMENT_SUPPORTED = 4,
  TC_ALIGNMENT_ERROR = 5,
} AlignStatus_t;

  /**
  * @brief Handle of a Position Control component
  */
typedef struct
{
  float MovementDuration;              /**< @brief Total duration of the programmed movement */
  float StartingAngle;                 /**< @brief Current mechanical position */
  float FinalAngle;                    /**< @brief Target mechanical position including start position */
  float AngleStep;                     /**< @brief Target mechanical position */
  float SubStep[6];                    /**< @brief Sub step interval time of acceleration and deceleration phases */
  float SubStepDuration;               /**< @brief Sub step time duration of sequence : acceleration / cruise / deceleration */
  float ElapseTime;                    /**< @brief Elapse time during trajectory movement execution */
  float SamplingTime;                  /**< @brief Sampling time at which the movement regulation is called (at 1/MEDIUM_FREQUENCY_TASK_RATE) */
  float Jerk;                          /**< @brief Angular jerk, rate of change of the angular acceleration with respect to time */
  float CruiseSpeed;                   /**< @brief Angular velocity during the time interval after acceleration and before deceleration */
  float Acceleration;                  /**< @brief Angular acceleration in rad/s^2 */
  float Omega;                         /**< @brief Estimated angular speed in rad/s */
  float OmegaPrev;                     /**< @brief Previous estimated angular speed of frame (N-1) */
  float Theta;                         /**< @brief Current angular position */
  float ThetaPrev;                     /**< @brief Angular position of frame (N-1) */
  uint8_t ReceivedTh;                  /**< @brief At startup of follow mode, need to receive two angles to compute the speed and acceleration */
  bool PositionControlRegulation;      /**< @brief Flag to activate the position control regulation */
  bool EncoderAbsoluteAligned;         /**< @brief Flag to indicate that absolute zero alignement is done */
  int16_t MecAngleOffset;              /**< @brief Store rotor mechanical angle offset */
  uint32_t TcTick;                     /**< @brief Tick counter in follow mode */
  float SysTickPeriod;                 /**< @brief Time base of follow mode */

  PosCtrlStatus_t PositionCtrlStatus;  /**< @brief Trajectory execution status */
  AlignStatus_t AlignmentCfg;          /**< @brief Indicates that zero index is supported for absolute alignment */
  AlignStatus_t AlignmentStatus;       /**< @brief Alignement procedure status */

  ENCODER_Handle_t *pENC;              /**< @brief Pointer on handler of the current instance of the encoder component */
  SpeednTorqCtrl_Handle_t *pSTC;       /**< @brief Speed and torque controller object used by the Position Regulator */
  PID_Handle_t *PIDPosRegulator;       /**< @brief PID controller object used by the Position Regulator */
} PosCtrl_Handle_t;

/* Exported functions ------------------------------------------------------- */

/* Initializes Trajectory Control component handler */
void TC_Init(PosCtrl_Handle_t *pHandle, PID_Handle_t *pPIDPosReg, SpeednTorqCtrl_Handle_t *pSTC,
             ENCODER_Handle_t *pENC);

/* Configures the trapezoidal speed trajectory */
bool TC_MoveCommand(PosCtrl_Handle_t *pHandle, float startingAngle, float angleStep, float movementDuration);

/* Follows an angular position command */
void TC_FollowCommand(PosCtrl_Handle_t *pHandle, float Angle);

/* Proceeds on the position control loop */
void TC_PositionRegulation(PosCtrl_Handle_t *pHandle);

/* Executes the programmed trajectory movement */
void TC_MoveExecution(PosCtrl_Handle_t *pHandle);

/* Updates the angular position */
void TC_FollowExecution(PosCtrl_Handle_t *pHandle);

/* Handles the alignment phase at starting before any position commands */
void TC_EncAlignmentCommand(PosCtrl_Handle_t *pHandle);

/* Checks if time allowed for movement is completed */
bool TC_RampCompleted(PosCtrl_Handle_t *pHandle);

/* Sets the absolute zero mechanical position */
void TC_EncoderReset(PosCtrl_Handle_t *pHandle);

/* Returns the current rotor mechanical angle, expressed in radiant */
float TC_GetCurrentPosition(PosCtrl_Handle_t *pHandle);

/* Returns the target rotor mechanical angle, expressed in radiant */
float TC_GetTargetPosition(PosCtrl_Handle_t *pHandle);

/* Returns the duration used to execute the movement, expressed in seconds */
float TC_GetMoveDuration(PosCtrl_Handle_t *pHandle);

/* Returns the status of the position control execution */
PosCtrlStatus_t TC_GetControlPositionStatus(PosCtrl_Handle_t *pHandle);

/* Returns the status after the rotor alignment phase */
AlignStatus_t TC_GetAlignmentStatus(PosCtrl_Handle_t *pHandle);

/* Increments Tick counter used in follow mode */
void TC_IncTick(PosCtrl_Handle_t *pHandle);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* TRAJCTRL_H */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
