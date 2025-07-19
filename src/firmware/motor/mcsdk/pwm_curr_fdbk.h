/**
  ******************************************************************************
  * @file    pwm_curr_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          PWM & Current Feedback component of the Motor Control SDK.
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
  * @ingroup pwm_curr_fdbk
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PWMNCURRFDBK_H
#define PWMNCURRFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"


/* Exported defines ------------------------------------------------------------*/

#define SECTOR_1  0U
#define SECTOR_2  1U
#define SECTOR_3  2U
#define SECTOR_4  3U
#define SECTOR_5  4U
#define SECTOR_6  5U
/*  @brief Used in calculation of Ia, Ib and Ic
  * 
  * See function PWMC_CalcPhaseCurrentsEst
  */
#define SQRT3FACTOR ((uint16_t)0xDDB4) /* = (16384 * 1.732051 * 2)*/

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @brief PWM & Current Sensing component handle type */
typedef struct PWMC_Handle PWMC_Handle_t;

/**
  * @brief Pointer on callback functions used by PWMC components
  *
  * This type is needed because the actual functions to use can change at run-time.
  *
  * See the following items:
  * - PWMC_Handle::pFctSwitchOffPwm
  * - PWMC_Handle::pFctSwitchOnPwm
  * - PWMC_Handle::pFctCurrReadingCalib
  * - PWMC_Handle::pFctRLDetectionModeEnable
  * - PWMC_Handle::pFctRLDetectionModeDisable
  *
  *
  */
typedef void (*PWMC_Generic_Cb_t)(PWMC_Handle_t *pHandle);

/**
  * @brief Pointer on the function provided by the PMWC component instance to get the phase current.
  *
  * This type is needed because the actual function to use can change at run-time
  * (See PWMC_Handle::pFctGetPhaseCurrents).
  *
  */
typedef void (*PWMC_GetPhaseCurr_Cb_t)(PWMC_Handle_t *pHandle, ab_t *Iab);

/**
  * @brief Pointer on the function provided by the PMWC component instance to set low sides ON.
  *
  * This type is needed because the actual function to use can change at run-time
  * (See PWMC_Handle::pFctTurnOnLowSides).
  *
  */
typedef void (*PWMC_TurnOnLowSides_Cb_t)(PWMC_Handle_t *pHandle, const uint32_t ticks);

/**
  * @brief Pointer on the function provided by the PMWC component instance to set the reference
  *        voltage for the over current protection.
  *
  * This type is needed because the actual function to use can change at run-time
  * (See PWMC_Handle::pFctOCPSetReferenceVoltage).
  *
  */
typedef void (*PWMC_SetOcpRefVolt_Cb_t)(PWMC_Handle_t *pHandle, uint16_t hDACVref);

/**
  * @brief Pointer on the functions provided by the PMWC component instance to set the ADC sampling
  *        point for each sectors.
  *
  * This type is needed because the actual function to use can change at run-time. See:
  * - PWMC_Handle::pFctSetADCSampPointSect1
  * - PWMC_Handle::pFctSetADCSampPointSect2
  * - PWMC_Handle::pFctSetADCSampPointSect3
  * - PWMC_Handle::pFctSetADCSampPointSect4
  * - PWMC_Handle::pFctSetADCSampPointSect5
  * - PWMC_Handle::pFctSetADCSampPointSect6
  *
  */
typedef uint16_t (*PWMC_SetSampPointSectX_Cb_t)(PWMC_Handle_t *pHandle);

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
  * @brief Pointer on the function provided by the PMWC component instance to set the PWM duty cycle
  *        in RL detection mode.
  *
  * This type is needed because the actual function to use can change at run-time
  * (See PWMC_Handle::pFctRLDetectionModeSetDuty).
  *
  */
typedef uint16_t (*PWMC_RLDetectSetDuty_Cb_t)(PWMC_Handle_t *pHandle, uint16_t hDuty);

/**
  * @brief Pointer on the function provided by the PMWC component instance to set the calibrated offsets
  *        in RL detection mode.
  *
  * This type is needed because the actual function to use can change at run-time
  * (See PWMC_Handle::pFctSetOffsetCalib).
  *
  */
typedef void (*PWMC_SetOffsetCalib_Cb_t)(PWMC_Handle_t *pHandle, PolarizationOffsets_t *offsets);

/**
  * @brief Pointer on the function provided by the PMWC component instance to get the calibrated offsets
  *        in RL detection mode.
  *
  * This type is needed because the actual function to use can change at run-time
  */
typedef void (*PWMC_GetOffsetCalib_Cb_t)(PWMC_Handle_t *pHandle, PolarizationOffsets_t *offsets);

/**
  * @brief This structure is used to handle the data of an instance of the PWM & Current Feedback component
  *
  */
struct PWMC_Handle
{
  /** @{ */
  PWMC_GetPhaseCurr_Cb_t
  pFctGetPhaseCurrents;                      /**< Pointer on the function the component instance uses to retrieve phase currents. */
  PWMC_Generic_Cb_t
  pFctSwitchOffPwm;                          /**< Pointer on the function the component instance uses to switch PWM off. */
  PWMC_Generic_Cb_t
  pFctSwitchOnPwm;                           /**< Pointer on the function the component instance uses to switch PWM on. */
  PWMC_Generic_Cb_t
  pFctCurrReadingCalib;                      /**< Pointer on the fct the component instance uses to calibrate the current reading ADC(s). */
  PWMC_TurnOnLowSides_Cb_t
  pFctTurnOnLowSides;                        /**< Pointer on the function the component instance uses to turn low sides on. */
  PWMC_SetSampPointSectX_Cb_t
  pFctSetADCSampPointSectX;                  /**< Pointer on the function the component instance uses to set the ADC sampling point. */
  PWMC_OverCurr_Cb_t
  pFctIsOverCurrentOccurred;                 /**< Pointer on the fct the component instance uses to return the over current status. */
  PWMC_SetOcpRefVolt_Cb_t
  pFctOCPSetReferenceVoltage;                /**< Pointer on the fct the component instance uses to set the over current ref voltage. */
  PWMC_Generic_Cb_t
  pFctRLDetectionModeEnable;                 /**< Pointer on the function the component instance uses to enable RL detection mode. */
  PWMC_Generic_Cb_t
  pFctRLDetectionModeDisable;                /**< Pointer on the function the component instance uses to disable RL detection mode. */
  PWMC_RLDetectSetDuty_Cb_t
  pFctRLDetectionModeSetDuty;                /**< Pointer on the fct the component instance uses to set the PWM duty cycle in RL
                                                  detection mode. */
  PWMC_SetOffsetCalib_Cb_t
  pFctSetOffsetCalib;                        /**< Pointer on the fct the component instance uses to set the calibrated offsets. */
  PWMC_GetOffsetCalib_Cb_t
  pFctGetOffsetCalib;                        /**< Pointer on the fct the component instance uses to get the calibrated offsets. */
  /** @} */
  int32_t   LPFIqBuf;                                  /**< LPF buffer for @f$ I_q @f$. */
  int32_t   LPFIdBuf;                                  /**< LPF buffer for @f$ I_d @f$. */
  uint16_t  hT_Sqrt3;                                  /**< Constant used by PWM algorithm (@f$\sqrt{3}@f$). */
  uint16_t  CntPhA;                                    /**< PWM Duty cycle for phase A. */
  uint16_t  CntPhB;                                    /**< PWM Duty cycle for phase B. */
  uint16_t  CntPhC;                                    /**< PWM Duty cycle for phase C. */
  uint16_t  SWerror;                                   /**< Contains status about SW error. */
  uint16_t  lowDuty;
  uint16_t  midDuty;
  uint16_t  highDuty;
  uint16_t  HighDutyStored;                            /**< Discontinuous PWM Store current Highest Duty for recovering.  */
  uint16_t  OffCalibrWaitTimeCounter;                  /**< Counter to wait fixed time before motor
                                                            current measurement offset calibration. */
  int16_t   Ia;                                        /**< Last @f$I_{A}@f$ measurement. */
  int16_t   Ib;                                        /**< Last @f$I_{B}@f$ measurement. */
  int16_t   Ic;                                        /**< Last @f$I_{C}@f$ measurement. */
  int16_t   IaEst;                                     /**< @f$I_{A}@f$ estimated. */
  int16_t   IbEst;                                     /**< @f$I_{B}@f$ estimated. */
  int16_t   IcEst;                                     /**< @f$I_{C}@f$ estimated. */
  int16_t   LPFIqd_const;                              /**< Low pass filter constant. */
  uint16_t  DTTest;                                    /**< Reserved. */
  uint16_t PWMperiod;                                  /**< PWM period expressed in timer clock cycles unit:
                                                         *  @f$hPWMPeriod = TimerFreq_{CLK} / F_{PWM}@f$    */
   uint16_t DTCompCnt;                                 /**< Half of Dead time expressed
                                                         *  in timer clock cycles unit:
                                                         *  @f$hDTCompCnt = (DT_s \cdot TimerFreq_{CLK})/2@f$ */
  uint16_t  Ton;                                       /**< Reserved. */
  uint16_t  Toff;                                      /**< Reserved. */

  uint8_t   Motor;                                     /**< Motor reference number. */
  uint8_t   AlignFlag;                                 /**< Phase current 0 is reliable, 1 is not. */
  uint8_t   Sector;                                    /**< Space vector sector number. */
  bool TurnOnLowSidesAction;                           /**< True if TurnOnLowSides action is active,
                                                            false otherwise. */
  bool      DPWM_Mode;                                 /**< Discontinuous PWM mode activation. */															  
  bool      RLDetectionMode;                           /**< True if enabled, false if disabled. */
  bool offsetCalibStatus;                              /**< True if offset calibration completed, false otherwise. */
  volatile  bool      useEstCurrent;                   /**< Estimated current flag. */

};

/**
  * @brief  Current reading calibration definition.
  */
typedef enum
{
  CRC_START, /**< Initializes the current reading calibration. */
  CRC_EXEC   /**< Executes the current reading calibration. */
} CRCAction_t;


/* Returns the phase current of the motor as read by the ADC (in s16A unit). */
void PWMC_GetPhaseCurrents(PWMC_Handle_t *pHandle, ab_t *Iab);

/* Converts input voltages @f$ V_{\alpha} @f$ and @f$ V_{\beta} @f$ into PWM duty cycles
 * and feed them to the inverter. */
uint16_t PWMC_SetPhaseVoltage(PWMC_Handle_t *pHandle, alphabeta_t Valfa_beta);

/* Switches PWM generation off, inactivating the outputs. */
void PWMC_SwitchOffPWM(PWMC_Handle_t *pHandle);

/* Enables PWM generation on the proper Timer peripheral. */
void PWMC_SwitchOnPWM(PWMC_Handle_t *pHandle);

/* Calibrates ADC current conversions by reading the offset voltage
 * present on ADC pins when no motor current is flowing in. */
bool PWMC_CurrentReadingCalibr(PWMC_Handle_t *pHandle, CRCAction_t action);

/* Switches power stage Low Sides transistors on. */
void PWMC_TurnOnLowSides(PWMC_Handle_t *pHandle, uint32_t ticks);

/* Sets the calibrated @p offsets for each of the phases in the @p pHandle handler. In case
 * of single shunt only phase A is relevant. */
void PWMC_SetOffsetCalib(PWMC_Handle_t *pHandle, PolarizationOffsets_t *offsets);

/* Gets the calibrated @p offsets for each of the phases in the @p pHandle handler. In case
 * of single shunt only phase A is relevant. */
void PWMC_GetOffsetCalib(PWMC_Handle_t *pHandle, PolarizationOffsets_t *offsets);


/* Checks if an overcurrent has occured since the last call to this function. */
uint16_t PWMC_CheckOverCurrent(PWMC_Handle_t *pHandle);

/* Sets the over current threshold through the DAC reference voltage. */
void PWMC_OCPSetReferenceVoltage(PWMC_Handle_t *pHandle, uint16_t hDACVref);

/* Retrieves the satus of TurnOnLowSides action. */
bool PWMC_GetTurnOnLowSidesAction(const PWMC_Handle_t *pHandle);

/* Enables Discontinuous PWM mode using the @p pHandle PWMC component. */
void PWMC_DPWM_ModeEnable(PWMC_Handle_t *pHandle);

/* Disables Discontinuous PWM mode using the @p pHandle PWMC component. */
void PWMC_DPWM_ModeDisable(PWMC_Handle_t *pHandle);

/* Returns the status of the Discontinuous PWM Mode stored in the @p pHandle PWMC component. */
bool PWMC_GetDPWM_Mode(PWMC_Handle_t *pHandle);

/* Enables the RL detection mode by calling the function in @p pHandle PWMC component. */
void PWMC_RLDetectionModeEnable(PWMC_Handle_t *pHandle);

/* Disables the RL detection mode by calling the function in @p pHandle PWMC component. */
void PWMC_RLDetectionModeDisable(PWMC_Handle_t *pHandle);

/* Sets the PWM duty cycle to apply in the RL Detection mode. */
uint16_t PWMC_RLDetectionModeSetDuty(PWMC_Handle_t *pHandle, uint16_t hDuty);

/* Sets the aligned motor flag. */
void PWMC_SetAlignFlag(PWMC_Handle_t *pHandle, uint8_t flag);

/* Sets the Callback that the PWMC component shall invoke to get phases current. */
void PWMC_RegisterGetPhaseCurrentsCallBack(PWMC_GetPhaseCurr_Cb_t pCallBack, PWMC_Handle_t *pHandle);

/* Sets the Callback that the PWMC component shall invoke to switch PWM generation off. */
void PWMC_RegisterSwitchOffPwmCallBack(PWMC_Generic_Cb_t pCallBack, PWMC_Handle_t *pHandle);

/* Sets the Callback that the PWMC component shall invoke to switch PWM generation on. */
void PWMC_RegisterSwitchonPwmCallBack(PWMC_Generic_Cb_t pCallBack, PWMC_Handle_t *pHandle);

/* Sets the Callback that the PWMC component shall invoke to execute a calibration of the current sensing system. */
void PWMC_RegisterReadingCalibrationCallBack(PWMC_Generic_Cb_t pCallBack, PWMC_Handle_t *pHandle);

/* Sets the Callback that the PWMC component shall invoke to turn low sides on. */
void PWMC_RegisterTurnOnLowSidesCallBack(PWMC_TurnOnLowSides_Cb_t pCallBack, PWMC_Handle_t *pHandle);

/* Sets the Callback that the PWMC component shall invoke to compute ADC sampling point. */
void PWMC_RegisterSampPointSectXCallBack(PWMC_SetSampPointSectX_Cb_t pCallBack, PWMC_Handle_t *pHandle);

/* Sets the Callback that the PWMC component shall invoke to check the overcurrent status. */
void PWMC_RegisterIsOverCurrentOccurredCallBack(PWMC_OverCurr_Cb_t pCallBack, PWMC_Handle_t *pHandle);

/* Sets the Callback that the PWMC component shall invoke to set the reference voltage for the overcurrent
  * protection. */
void PWMC_RegisterOCPSetRefVoltageCallBack(PWMC_SetOcpRefVolt_Cb_t pCallBack, PWMC_Handle_t *pHandle);

/* Sets the Callback that the PWMC component shall invoke to enable the R/L detection mode. */
void PWMC_RegisterRLDetectionModeEnableCallBack(PWMC_Generic_Cb_t pCallBack, PWMC_Handle_t *pHandle);

/* Sets the Callback that the PWMC component shall invoke to disable the R/L detection mode. */
void PWMC_RegisterRLDetectionModeDisableCallBack(PWMC_Generic_Cb_t pCallBack, PWMC_Handle_t *pHandle);

/* Sets the Callback that the PWMC component shall invoke to set the duty cycle for the R/L detection mode */
void PWMC_RegisterRLDetectionModeSetDutyCallBack(PWMC_RLDetectSetDuty_Cb_t pCallBack, PWMC_Handle_t *pHandle);


/* Used to clear variables in CPWMC. */
void PWMC_Clear(PWMC_Handle_t *pHandle);

/* Converts input currents components Iqd into estimated currents Ia, Ib and Ic. */
void PWMC_CalcPhaseCurrentsEst(PWMC_Handle_t *pHandle, qd_t Iqd, int16_t hElAngledpp);

/* Converts input voltage components @f$ V_{\alpha} @f$ and @f$ V_{\beta} @f$ into duty cycles
 * and feed them to the inverter with overmodulation function. */
uint16_t PWMC_SetPhaseVoltage_OVM(PWMC_Handle_t *pHandle, alphabeta_t Valfa_beta);

/**
  * @}
  */

/**
  * @}
  */
 
/* Executes a regular conversion using the first ADC used for current reading.
 * The function is not re-entrant (it cannot executed twice at the same time).
 * Returns 0xFFFF in case of conversion error. */
uint16_t PWMC_ExecRegularConv(PWMC_Handle_t *pHandle, uint8_t bChannel);

/* Sets the specified sampling time for the specified ADC channel on the first ADC used for current
 * reading. Must be called once for each channel utilized by user. */
void PWMC_ADC_SetSamplingTime(PWMC_Handle_t *pHandle, ADConv_t ADConv_struct);

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* PWMNCURRFDBK_H */

/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
