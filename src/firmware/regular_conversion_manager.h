
/**
  ******************************************************************************
  * @file    regular_conversion_manager.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          regular_conversion_manager component of the Motor Control SDK.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef REGULAR_CONVERSION_MANAGER_H
#define REGULAR_CONVERSION_MANAGER_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup RCM
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief RegConv_t contains all the parameters required to execute a regular conversion
  *
  * it is used by all regular_conversion_manager's client
  *
  */
typedef struct
{
  ADC_TypeDef *regADC;
  uint8_t  channel;
  uint32_t samplingTime;
} RegConv_t;

/**
 * @brief Conversion states
 */
typedef enum
{
  RCM_USERCONV_IDLE,        /**< @brief No conversion currently scheduled */
  RCM_USERCONV_REQUESTED,   /**< @brief A conversion is scheduled for execution */
  RCM_USERCONV_EOC          /**< @brief A conversion has completed and the value is ready */
}RCM_UserConvState_t;

typedef void (*RCM_exec_cb_t)(uint8_t handle, uint16_t data, void *UserData);

/* Exported functions ------------------------------------------------------- */

/*  Function used to register a regular conversion */
uint8_t RCM_RegisterRegConv(RegConv_t *regConv);

/*  Function used to register a regular conversion with a callback attached*/
uint8_t RCM_RegisterRegConv_WithCB(RegConv_t *regConv, RCM_exec_cb_t fctCB, void *data);

/*  Function used to execute an already registered regular conversion */
uint16_t RCM_ExecRegularConv(uint8_t handle);

/* select the handle conversion to be executed during the next call to RCM_ExecUserConv */
bool RCM_RequestUserConv(uint8_t handle);

/* return the latest user conversion value*/
uint16_t RCM_GetUserConv(void);

/* Must be called by MC_TASK only to grantee proper scheduling*/
void RCM_ExecUserConv(void);

/* return the state of the user conversion state machine*/
RCM_UserConvState_t RCM_GetUserConvState(void);

/* Function used to un-schedule a regular conversion exectuted after current sampling in HF task */
bool RCM_PauseRegularConv(uint8_t handle);

/* non blocking function to start conversion inside HF task */
void RCM_ExecNextConv(void);

/* non blocking function used to read back already started regular conversion*/
void RCM_ReadOngoingConv(void);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* REGULAR_CONVERSION_MANAGER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
