/**
  ******************************************************************************
  * @file    sto_speed_pos_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains definitions and functions prototypes common to all
  *          State Observer based Speed & Position Feedback components of the Motor
  *          Control SDK (the CORDIC and PLL implementations).
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
  * @ingroup SpeednPosFdbk
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STO_SPEEDNPOSFDBK_H
#define STO_SPEEDNPOSFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "speed_pos_fdbk.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk
  * @{
  */


/** @brief PWM & Current Sensing component handle type */
typedef struct STO_Handle STO_Handle_t;

typedef void (*STO_ForceConvergency1_Cb_t)(STO_Handle_t *pHandle);
typedef void (*STO_ForceConvergency2_Cb_t)(STO_Handle_t *pHandle);
typedef void (*STO_OtfResetPLL_Cb_t)(STO_Handle_t *pHandle);
typedef bool (*STO_SpeedReliabilityCheck_Cb_t)(const STO_Handle_t *pHandle);

/**
  * @brief Handle of the Speed and Position Feedback STO component.
  *
  */
struct STO_Handle
{
  SpeednPosFdbk_Handle_t           *_Super;                               /**< @brief Speed and torque component handler. */
  STO_ForceConvergency1_Cb_t       pFctForceConvergency1;                 /**< @brief Function to force observer convergence. */
  STO_ForceConvergency2_Cb_t       pFctForceConvergency2;                 /**< @brief Function to force observer convergence. */
  STO_OtfResetPLL_Cb_t             pFctStoOtfResetPLL;                    /**< @brief Function to reset on the fly start-up. */
  STO_SpeedReliabilityCheck_Cb_t   pFctSTO_SpeedReliabilityCheck;         /**< @brief Function to check the speed reliability. */
};

/**
  * @}
  */

/**
  * @}
  */


#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*STO_SPEEDNPOSFDBK_H*/

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
