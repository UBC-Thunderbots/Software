/**
  ******************************************************************************
  * @file    motorcontrol.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Motor Control Subsystem initialization functions.
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
  * @ingroup MCInterface
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include "mc_config.h"
#include "parameters_conversion.h"
#include "mc_api.h"

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup MCInterface
  * @{
  */

/* Initializes the Motor Control Subsystem */
void MX_MotorControl_Init(void);

/* Do not remove the definition of this symbol. */
#define MC_HAL_IS_USED
/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* MOTORCONTROL_H */
/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
