/**
  ******************************************************************************
  * @file    mc_app_hooks.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements default motor control app hooks.
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
  * @ingroup MCAppHooks
  */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "mc_app_hooks.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup MCTasks
  * @{
  */

/**
 * @defgroup MCAppHooks Motor Control Applicative hooks
 * @brief User defined functions that are called in the Motor Control tasks
 *
 *
 * @{
 */

/**
 * @brief Hook function called right after the Medium Frequency Task
 *
 *
 *
 */
__weak void MC_APP_BootHook(void)
{
  /*
   * This function can be overloaded or the application can inject
   * code into it that will be executed at the end of MCboot().
   */

/* USER CODE BEGIN BootHook */

/* USER CODE END BootHook */
}

/**
 * @brief Hook function called right after the Medium Frequency Task
 *
 *
 *
 */
__weak void MC_APP_PostMediumFrequencyHook_M1(void)
{
  /*
   * This function can be overloaded or the application can inject
   * code into it that will be executed right after the Medium
   * Frequency Task of Motor 1.
   */

/* USER SECTION BEGIN PostMediumFrequencyHookM1 */

/* USER SECTION END PostMediumFrequencyHookM1 */
}

/** @} */

/** @} */

/** @} */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
