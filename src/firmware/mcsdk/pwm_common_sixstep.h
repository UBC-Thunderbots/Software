/**
  ******************************************************************************
  * @file    pwm_common_sixstep.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          six-step PWM component of the Motor Control SDK.
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
  * @ingroup pwm_curr_fdbk_6s
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PWMNCOMMON_SIXSTEP_H
#define PWMNCOMMON_SIXSTEP_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#define NB_CONVERSIONS 16u

#define S16_120_PHASE_SHIFT (int16_t)(65536/3)
#define S16_60_PHASE_SHIFT  (int16_t)(65536/6)

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/** @addtogroup pwm_curr_fdbk_6s
  * @{
  */
/* Exported defines ------------------------------------------------------------*/

#define STEP_1  0U
#define STEP_2  1U
#define STEP_3  2U
#define STEP_4  3U
#define STEP_5  4U
#define STEP_6  5U

/* Exported defines ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/** @brief PWMC component handle type */
typedef struct PWMC_Handle PWMC_Handle_t;

/**
  * @brief Pointer on callback functions used by PWMC components
  *
  * This type is needed because the actual functions to use can change at run-time.
  *
  * See the following items:
  * - PWMC_Handle::pFctSwitchOffPwm
  * - PWMC_Handle::pFctSwitchOnPwm

  *
  *
  */
typedef void (*PWMC_Generic_Cb_t)(PWMC_Handle_t *pHandle);

/**
  * @brief Pointer on the function provided by the PMWC component instance to set low sides ON.
  *
  * This type is needed because the actual function to use can change at run-time
  * (See PWMC_Handle::pFctTurnOnLowSides).
  *
  */
typedef void (*PWMC_TurnOnLowSides_Cb_t)(PWMC_Handle_t *pHandle, const uint32_t ticks);


/**
  * @brief Pointer on the function provided by the PMWC component instance to set the trigger point 
  *        of the ADC.
  *
  * This type is needed because the actual function to use can change at run-time
  * (See PWMC_Handle::pFctOCPSetReferenceVoltage).
  *
  */
typedef void (*PWMC_SetADCTriggerChannel_Cb_t)(PWMC_Handle_t *pHandle, uint16_t hDACVref);


/**
  * @brief Pointer on the function provided by the PMWC component instance to check if an over current
  *        condition has occured.
  *
  * This type is needed because the actual function to use can change at run-time
  * (See PWMC_Handle::pFctIsOverCurrentOccurred).
  *
  */
typedef uint16_t (*PWMC_OverCurr_Cb_t)(PWMC_Handle_t *pHandle);

/**
  * @brief Pointer on the function provided by the PMWC component instance to check optional 
  *        modulation features flag (fast demagnetization and quasi-synchronous rectification)
  *
  */
typedef uint8_t (*PWMC_ModFlag_Cb_t)(PWMC_Handle_t *pHandle);

/**
  * @brief This structure is used to handle the status change of optional modulation features
  *
  */
typedef enum
{
  NO_REQUEST,
  ENABLE_FAST_DEMAG,
  DISABLE_FAST_DEMAG,
  ENABLE_QUASI_SYNCH,
  DISABLE_QUASI_SYNCH
} PWMTableUpdate_t;

/**
  * @brief This structure is used to handle the data of an instance of the PWM  component
  *
  */
struct PWMC_Handle
{
  /** @{ */
  PWMC_Generic_Cb_t
  pFctSwitchOffPwm;                        /**< pointer on the function the component instance used to switch PWM off */
  PWMC_Generic_Cb_t
  pFctSwitchOnPwm;                         /**< pointer on the function the component instance used to switch PWM on */
  PWMC_SetADCTriggerChannel_Cb_t           /**< pointer on the function the component instance used to set the trigger point of the ADC */
  pFctSetADCTriggerChannel;
  PWMC_TurnOnLowSides_Cb_t
  pFctTurnOnLowSides;                      /**< pointer on the function the component instance used to turn low sides on */
  PWMC_OverCurr_Cb_t
  pFctIsOverCurrentOccurred;               /**< pointer on the fct the component instance used to return the over current status */
  PWMC_ModFlag_Cb_t
  pGetFastDemagFlag;                       /**< pointer on the fct the component instance used to return the fast demag status */
  PWMC_ModFlag_Cb_t
  pGetQuasiSynchFlag;                      /**< pointer on the fct the component instance used to return the quasi-Synch rectification status */
  /** @} */
  uint16_t CntPh;                                    /**< PWM Duty cycle phase*/
  uint16_t StartCntPh;                               /**< Start-up PWM Duty cycle phase*/
  uint16_t ADCTriggerCnt;                            /**< Timer output trigger point used for ADC triggering */
  uint16_t SWerror;                                  /**< Contains status about SW error */
  uint16_t PWMperiod;                                /**< PWM period expressed in timer clock cycles unit:
                                                          *  @f$hPWMPeriod = TimerFreq_{CLK} / F_{PWM}@f$    */
  uint16_t DTCompCnt;                                /**< Half of Dead time expressed
                                                          *  in timer clock cycles unit:
                                                          *  @f$hDTCompCnt = (DT_s \cdot TimerFreq_{CLK})/2@f$ */

  uint8_t  Motor;                                    /**< Motor reference number */
  int16_t  AlignFlag;                                /*!< phase current 0 is reliable, 1 is bad */
  uint8_t  NextStep;                                 /**< Step number to be applied the step number */
  uint8_t  Step;                                     /**< Step number */
  uint16_t DemagCounterThreshold;
  int16_t  hElAngle;
  bool TurnOnLowSidesAction;                         /**< true if TurnOnLowSides action is active,
                                                              false otherwise. */
  PWMTableUpdate_t ModUpdateReq;	                 /**< Request flag of optional modulation features status  */														  
};

/* Exported functions --------------------------------------------------------*/

/* Switches the PWM generation off, setting the outputs to inactive */
void PWMC_SwitchOffPWM(PWMC_Handle_t *pHandle);

/* Switches the PWM generation on */
void PWMC_SwitchOnPWM(PWMC_Handle_t *pHandle);

/* Set the trigger instant of the ADC for Bemf acquisition*/
void PWMC_SetADCTriggerChannel( PWMC_Handle_t * pHdl, uint16_t SamplingPoint );

/* Turns low sides on. This function is intended to be used for
 *  charging boot capacitors of driving section. It has to be called on each
 *  motor start-up when using high voltage drivers. */
void PWMC_TurnOnLowSides(PWMC_Handle_t * pHandle, uint32_t ticks);

/* Checks if an over current occurred since last call. */
uint16_t PWMC_CheckOverCurrent(PWMC_Handle_t *pHandle);

/* Retrieves the status of the "TurnOnLowSides" action. */
bool PWMC_GetTurnOnLowSidesAction( PWMC_Handle_t * pHandle );

/* It is used to set the align motor flag.*/
void PWMC_SetAlignFlag(PWMC_Handle_t *pHandle, int16_t flag);

/* Sets the Callback that the PWMC component shall invoke to switch off PWM
 *        generation. */
void PWMC_RegisterSwitchOffPwmCallBack(PWMC_Generic_Cb_t pCallBack, PWMC_Handle_t *pHandle);

/* Sets the Callback that the PWMC component shall invoke to switch on PWM
 *        generation. */
void PWMC_RegisterSwitchonPwmCallBack(PWMC_Generic_Cb_t pCallBack, PWMC_Handle_t *pHandle);

/* Sets the Callback that the PWMC component shall invoke to turn on low sides. */
void PWMC_RegisterTurnOnLowSidesCallBack(PWMC_TurnOnLowSides_Cb_t pCallBack, PWMC_Handle_t *pHandle);

/* Sets the Callback that the PWMC component shall invoke to the over current status. */
void PWMC_RegisterIsOverCurrentOccurredCallBack(PWMC_OverCurr_Cb_t pCallBack, PWMC_Handle_t *pHandle);

/* It is used to clear the variable in CPWMC.  */
void PWMC_Clear(PWMC_Handle_t *pHandle);

/* It forces the Fast Demag interval to the passed value */
void PWMC_ForceFastDemagTime(PWMC_Handle_t * pHdl, uint16_t constFastDemagTime );

/* It enables/disables the Fast Demag feature */
void PWMC_SetFastDemagState(PWMC_Handle_t * pHandle, uint8_t State );

/* It enables/disables the Qusi Synch feature */
void PWMC_SetQuasiSynchState(PWMC_Handle_t * pHandle, uint8_t State );

/* It returns the Fast Demag feature status */
uint8_t PWMC_GetFastDemagState(PWMC_Handle_t * pHandle );

/* It returns the Quasi Synch feature status */
uint8_t PWMC_GetQuasiSynchState(PWMC_Handle_t * pHandle );

/* It converts the motor electrical angle to the corresponding step in the six-step sequence  */
uint8_t PWMC_ElAngleToStep( PWMC_Handle_t * pHandle );
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* PWMNCOMMON_SIXSTEP_H*/

/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
