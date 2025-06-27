/**
  ******************************************************************************
  * @file    pwmc_3pwm.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          pwmc_3pwm component of the Motor Control SDK.
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
  * @ingroup pwmc_3pwm
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PWMC_3PWM_H
#define __PWMC_3PWM_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "pwm_common_sixstep.h"

/**
 * @addtogroup MCSDK
 * @{
 */

/**
 * @addtogroup pwm_curr_fdbk_6s
 * @{
 */

/**
 * @addtogroup pwmc_3pwm
 * @{
 */

/* Exported constants --------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/

/**
  * @brief  ThreePwm parameters definition
  */
typedef struct
{
  TIM_TypeDef * TIMx;                  /*!< It contains the pointer to the timer
                                           used for PWM generation. */
  uint8_t  RepetitionCounter;          /*!< It expresses the number of PWM
                                            periods to be elapsed before compare
                                            registers are updated again. In
                                            particular:
                                            RepetitionCounter= (2* #PWM periods)-1*/
  uint32_t OCPolarity;                 /*!< timer channel output polarity.*/
  GPIO_TypeDef * pwm_en_u_port;        /*!< phase u enable driver signal GPIO port */
  uint16_t pwm_en_u_pin;               /*!< phase u enable driver signal pin */
  GPIO_TypeDef * pwm_en_v_port;        /*!< phase v enable driver signal GPIO port */
  uint16_t pwm_en_v_pin;               /*!< phase v enable driver signal pin */
  GPIO_TypeDef * pwm_en_w_port;        /*!< phase w enable driver signal GPIO port */
  uint16_t pwm_en_w_pin;               /*!< phase w enable driver signal pin */
} ThreePwm_Params_t;

/**
  * @brief  Handle structure of the PWMC_ThreePwm Component
  */
typedef struct
{
  PWMC_Handle_t _Super;        /*!<   */
  bool OverCurrentFlag;         /*!< This flag is set when an overcurrent occurs.*/
  bool OverVoltageFlag;         /*!< This flag is set when an overvoltage occurs.*/
  bool BrakeActionLock;         /*!< This flag is set to avoid that brake action is
                                 *   interrupted.*/
  bool FastDemag;         		/*!< This flag is set when the fast-demagmatization is activated.*/
  uint32_t NegOCPolarity;    /*!< Channel output opposite polarity.*/
  uint32_t NegOCNPolarity;   /*!< Complementary channel output opposite polarity. */
  uint16_t DemagCounter;
  ThreePwm_Params_t const * pParams_str;
} PWMC_ThreePwm_Handle_t;

/* Exported functions ------------------------------------------------------- */

/**
  * It initializes TIMx and NVIC
  */
void ThreePwm_Init( PWMC_ThreePwm_Handle_t * pHandle );

/**
  * It updates the stored duty cycle variable.
  */
void PWMC_SetPhaseVoltage( PWMC_Handle_t * pHandle, uint16_t DutyCycle );

/**
  * It writes the duty cycle into shadow timer registers.
  */
void ThreePwm_LoadNextStep( PWMC_ThreePwm_Handle_t * pHandle, int16_t Direction );

/**
  * It uploads the duty cycle into timer registers.
  */
bool ThreePwm_ApplyNextStep( PWMC_ThreePwm_Handle_t * pHandle );

/**
  * It resets the polarity of the timer PWM channel outputs to default
  */
void ThreePwm_ResetOCPolarity( PWMC_ThreePwm_Handle_t * pHandle );

/**
  * It turns on low sides switches. This function is intended to be
  * used for charging boot capacitors of driving section. It has to be
  * called each motor start-up when using high voltage drivers
  */
void ThreePwm_TurnOnLowSides( PWMC_Handle_t * pHdl, uint32_t ticks);

/**
  * This function enables the PWM outputs
  */
void ThreePwm_SwitchOnPWM( PWMC_Handle_t * pHdl );

/**
  * It disables PWM generation on the proper Timer peripheral acting on
  * MOE bit and reset the TIM status
  */
void ThreePwm_SwitchOffPWM( PWMC_Handle_t * pHdl );

/**
  * It sets the capcture compare of the timer channel used for ADC triggering
  */
void ThreePwm_SetADCTriggerChannel( PWMC_Handle_t * pHdl, uint16_t SamplingPoint );

/**
  * It is used to check if an overcurrent occurred since last call.
  */
uint16_t ThreePwm_IsOverCurrentOccurred( PWMC_Handle_t * pHdl );

/**
 * It contains the Break event interrupt
 */
void * ThreePwm_BRK_IRQHandler( PWMC_ThreePwm_Handle_t * pHandle );

/**
  * It is used to return the fast demag flag.
 */
uint8_t ThreePwm_FastDemagFlag( PWMC_Handle_t * pHdl );

/**
  * @}
  */

/**
  * @}
  */

/** @} */
#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*__PWMC_3PWM_H*/

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
