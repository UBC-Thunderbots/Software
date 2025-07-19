/**
  ******************************************************************************
  * @file    mc_app_hooks.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements tasks definition.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MC_APP_HOOKS_H
#define MC_APP_HOOKS_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup MCTasks
  * @{
  */

/** @addtogroup MCAppHooks
 * @{
 */

/* Exported functions ------------------------------------------------------- */

/* Hook function called at end of MCboot() */
void MC_APP_BootHook(void);

/* Hook function called right after the Medium Frequency Task of Motor 1 */
void MC_APP_PostMediumFrequencyHook_M1(void);

/** @} */

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* MC_APP_HOOKS_H */

/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
