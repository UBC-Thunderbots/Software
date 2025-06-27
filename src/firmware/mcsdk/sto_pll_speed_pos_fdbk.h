/**
  ******************************************************************************
  * @file    sto_pll_speed_pos_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          State Observer + PLL Speed & Position Feedback component of the Motor
  *          Control SDK.
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
  * @ingroup SpeednPosFdbk_STO_PLL
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STO_PLL_SPEEDNPOSFDBK_H
#define STO_PLL_SPEEDNPOSFDBK_H

/* Includes ------------------------------------------------------------------*/
#include "speed_pos_fdbk.h"
#include "sto_speed_pos_fdbk.h"
#include "pid_regulator.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk_STO_PLL
  * @{
  */


/**
  * @brief Handle of the Speed and Position Feedback STO PLL component.
  *
  */

typedef struct
{
  SpeednPosFdbk_Handle_t _Super;          /**< @brief Speed and torque component handler. */

  int16_t  hC1;                           /*!< @brief State observer constant @f$ C_1 @f$.
                                            *
                                            *  Can be computed as : @f$ (F_1 × R_s) / (L_s × State Observer Execution Rate) @f$ \n
                                            *  @f$ F_1 @f$ in Hertz [**Hz**]. \n
                                            *  @f$ R_s @f$ in Ohm @f$[\Omega]@f$. \n
                                            *  @f$ L_s @f$ in Henry [**H**]. \n
                                            *  State Observer Execution Rate in Hertz [**Hz**].
                                            */
  int16_t  hC2;                           /**< @brief State observer constant @f$ C_2 @f$.
                                            *
                                            *  Can be computed as : @f$ (F_1 × K_1) / (State Observer Execution Rate) @f$ \n
                                            *  @f$ F_1 @f$ in Hertz [**Hz**]. \n 
                                            *  @f$ K_1 @f$ being one of the two observer gains. \n
                                            *  State Observer Execution Rate in Hertz [**Hz**].
                                            */
  int16_t  hC3;                           /**< @brief State observer constant @f$ C_3 @f$.
                                            *
                                            *  Can be computed as : @f$ (F_1 × Max Application Speed × Motor Bemf Constant × √2) / (L_S × Max Measurable Current × State Observer Execution Rate) @f$ \n
                                            *  @f$ F_1 @f$ in Hertz [**Hz**]. \n
                                            *  Max Application Speed in Rotation per minute [**rpm**]. \n
                                            *  Motor Bemf Constant in Voltage line to line root mean square per kilo-rpm [**Vllrms/krpm**]. \n
                                            *  @f$ L_s @f$ in Henry [**H**]. \n
                                            *  Max Measurable Current in Ampere [**A**]. \n
                                            *  State Observer Execution Rate in Hertz [**Hz**].
                                            */
  int16_t  hC4;                           /**< @brief State Observer constant @f$ C_4 @f$.
                                            *
                                            *  Can be computed as @f$ K_2 × Max Measurable Current / (Max Application Speed × Motor Bemf Constant × √2 × F_2 × State Observer Execution Rate) @f$ \n
                                            *  @f$ K_2 @f$ being one of the two observer gains. \n
                                            *  Max Measurable Current in Ampere [**A**]. \n
                                            *  Max Application Speed in Rotation per minute [**rpm**]. \n
                                            *  Motor Bemf Constant in Voltage line to line root mean square per kilo-rpm [**Vllrms/krpm**]. \n
                                            *  State Observer Execution Rate in Hertz [**Hz**]. \n
                                            */
  int16_t  hC5;                           /**< @brief State observer constant @f$ C_5 @f$. 
                                            *
                                            *  Can be computed as @f$ F_1 × Max Measurable Voltage / (2 × L_s × Max Measurable Current × State Observer Execution Rate) @f$ \n
                                            *  @f$ F_1 @f$ in Hertz [**Hz**]. \n
                                            *  Max Measurable Voltage in Volt [**V**]. \n
                                            *  @f$ L_s @f$ in Henry [**H**]. \n
                                            *  Max Measurable Current in Ampere [**A**]. \n
                                            *  State Observer Execution Rate in Hertz [**Hz**].
                                            */
  int16_t  hC6;                           /**< @brief State observer constant @f$ C_6 @f$. Computed with a specific procedure starting from the other constants. */
  
  int16_t  hF1;                           /**< @brief State observer scaling factor @f$ F_1 @f$. */
  int16_t  hF2;                           /**< @brief State observer scaling factor @f$ F_2 @f$. */
  int16_t  hF3;                           /**< @brief State observer scaling factor @f$ F_3 @f$. */
  uint16_t F3POW2;                        /**< @brief State observer scaling factor @f$ F_3 @f$ expressed as power of 2.
                                            *
                                            *  E.g. If gain divisor is 512 the value must be 9 because @f$ 2^9 = 512 @f$.
                                            */
  PID_Handle_t PIRegulator;               /**< @brief PI regulator component handle, used for PLL implementation */
  int32_t Ialfa_est;                      /**< @brief Estimated @f$ I_{alpha} @f$ current. */
  int32_t Ibeta_est;                      /**< @brief Estimated @f$ I_{beta} @f$ current. */
  int32_t wBemf_alfa_est;                 /**< @brief Estimated Bemf alpha. */
  int32_t wBemf_beta_est;                 /**< @brief Estimated Bemf beta. */
  int16_t hBemf_alfa_est;                 /**< @brief Estimated Bemf alpha in int16_t format. */
  int16_t hBemf_beta_est;                 /**< @brief Estimated Bemf beta in int16_t format. */
  int16_t Speed_Buffer[64];               /**< @brief Estimated speed FIFO, it contains latest SpeedBufferSizeDpp speed measurements [**DPP**]. */
  uint8_t Speed_Buffer_Index;             /**< @brief Index of latest estimated speed in buffer Speed_Buffer[]. */
  bool IsSpeedReliable;                   /**< @brief Estimated speed reliability information.
                                            *
                                            *  Updated during speed average computation in STO_PLL_CalcAvrgMecSpeedUnit,
                                            *  True if the speed measurement variance is lower than threshold VariancePercentage.
                                            */
  uint8_t ConsistencyCounter;             /**< @brief Counter of passed tests for start-up validation. */
  uint8_t ReliabilityCounter;             /**< @brief Counter for checking reliability of Bemf and speed. */
  bool IsAlgorithmConverged;              /**< @brief Observer convergence flag. */
  bool IsBemfConsistent;                  /**< @brief Reliability of estimated Bemf flag.
                                            *
                                            *  Updated by STO_PLL_CalcAvrgMecSpeedUnit, set to true when observed Bemfs are consistent with expectation.
                                            */
  int32_t Obs_Bemf_Level;                 /**< @brief Magnitude of observed Bemf level squared. */
  int32_t Est_Bemf_Level;                 /**< @brief Magnitude of estimated Bemf Level squared based on speed measurement. */
  bool EnableDualCheck;                   /**< @brief Enable additional reliability check based on observed Bemf. */
  int32_t DppBufferSum;                   /**< @brief Sum of speed buffer elements [**DPP**]. */
  int16_t SpeedBufferOldestEl;            /**< @brief Oldest element of the speed buffer. */

  uint8_t SpeedBufferSizeUnit;            /**< @brief Depth of FIFO used to calculate the average estimated speed exported by SPD_GetAvrgMecSpeedUnit. 
                                            *         Must be an integer number in range[1..64].
                                            */
  uint8_t SpeedBufferSizeDpp;             /**< @brief Depth of FIFO used to calculate both average estimated speed exported by SPD_GetElSpeedDpp and state observer equations.
                                            *         Must be an integer number between 1 and SpeedBufferSizeUnit
                                            */
  uint16_t VariancePercentage;            /**< @brief Maximum allowed variance of speed estimation. */

  uint8_t SpeedValidationBand_H;          /**< @brief Upper bound below which the estimated speed is still acceptable 
                                            *         despite exceeding the force stator electrical frequency 
                                            *         during start-up. The measurement unit is 1/16 of forced
                                            *         speed.
                                            */
  uint8_t SpeedValidationBand_L;          /**< @brief Lower bound above which the estimated speed is still acceptable 
                                            *         despite subceeding the force stator electrical frequency 
                                            *         during start-up. The measurement unit is 1/16 of forced
                                            *         speed.
                                            */
  uint16_t MinStartUpValidSpeed;          /**< @brief Absolute value of minimum mechanical
                                            *         speed required to validate the start-up.
                                            *         Expressed in the unit defined by #SPEED_UNIT.
                                            */
  uint8_t StartUpConsistThreshold;        /**< @brief Number of consecutive tests on speed
                                            *         consistency to be passed before
                                            *         validating the start-up.
                                            */
  uint8_t Reliability_hysteresys;         /**< @brief Number of failed consecutive reliability tests
                                            *         before returning a speed check fault to _Super.bSpeedErrorNumber.
                                            */
  uint8_t BemfConsistencyCheck;           /**< @brief Degree of consistency of the observed Bemfs. \n
                                            *         Must be an integer number ranging from 1 (very high
                                            *         consistency) down to 64 (very poor consistency).
                                            */
  uint8_t BemfConsistencyGain;            /**< @brief Gain to be applied when checking Bemfs consistency. \n
                                            *         Default value is 64 (neutral), max value 105
                                            *         (x1.64 amplification), min value 1 (/64 attenuation).
                                            */
  uint16_t MaxAppPositiveMecSpeedUnit;    /**< @brief Maximum positive value of rotor speed. \n
                                            *         Expressed in the unit defined by #SPEED_UNIT. 
                                            *         Can be x1.1 greater than max application speed.
                                            */
  uint16_t F1LOG;                         /**< @brief @f$ F_1 @f$ gain divisor expressed as power of 2.
                                            *
                                            *  E.g. if gain divisor is 512 the value must be 9 because @f$ 2^9 = 512 @f$.
                                            */
  uint16_t F2LOG;                         /**< @brief @f$ F_2 @f$ gain divisor expressed as power of 2.
                                            *
                                            *  E.g. if gain divisor is 512 the value must be 9 because @f$ 2^9 = 512 @f$.
                                            */
  uint16_t SpeedBufferSizeDppLOG;         /**< @brief bSpeedBufferSizedpp expressed as power of 2.
                                            *
                                            *  E.g. if gain divisor is 512 the value must be 9 because @f$ 2^9 = 512 @f$.
                                            */
  bool ForceConvergency;                  /**< @brief Variable to force observer convergence. */
  bool ForceConvergency2;                 /**< @brief Variable to force observer convergence. */

  int8_t hForcedDirection;                /**< @brief Variable to force rotation direction. */

} STO_PLL_Handle_t;


/* Exported functions ------------------------------------------------------- */

/* Initializes the handler of STate Observer (STO) PLL component. */
void STO_PLL_Init(STO_PLL_Handle_t *pHandle);

/* Necessary empty return to implement fictitious IRQ_Handler. */
void STO_PLL_Return(STO_PLL_Handle_t *pHandle, uint8_t flag);

/* Clears state observer component by re-initializing private variables in the handler. */
void STO_PLL_Clear(STO_PLL_Handle_t *pHandle);

/* Calculates the estimated electrical angle. */
int16_t STO_PLL_CalcElAngle(STO_PLL_Handle_t *pHandle, Observer_Inputs_t *pInputs);

/* Computes and returns the average mechanical speed. */
bool STO_PLL_CalcAvrgMecSpeedUnit(STO_PLL_Handle_t *pHandle, int16_t *pMecSpeedUnit);

/* Resets the PLL integral term during on-the-fly startup. */
void STO_OTF_ResetPLL(STO_Handle_t *pHandle);

/* Resets the PLL integral term. */
void STO_ResetPLL(STO_PLL_Handle_t *pHandle);

/* Checks if the state observer algorithm converged. */
bool STO_PLL_IsObserverConverged(STO_PLL_Handle_t *pHandle, int16_t *phForcedMecSpeedUnit);

/* Computes and updates the average electrical speed. */
void STO_PLL_CalcAvrgElSpeedDpp(STO_PLL_Handle_t *pHandle);

/* Exports estimated Bemf alpha-beta from the handler. */
alphabeta_t STO_PLL_GetEstimatedBemf(STO_PLL_Handle_t *pHandle);

/* Exports from the handler the stator current alpha-beta as estimated by state observer. */
alphabeta_t STO_PLL_GetEstimatedCurrent(STO_PLL_Handle_t *pHandle);

/* Stores in the handler the new values for observer gains. */
void STO_PLL_SetObserverGains(STO_PLL_Handle_t *pHandle, int16_t hhC1, int16_t hhC2);

/* Exports current observer gains from the handler to parameters hhC2 and hhC4. */
void STO_PLL_GetObserverGains(STO_PLL_Handle_t *pHandle, int16_t *phC2, int16_t *phC4);

/* Exports the current PLL gains from the handler to parameters pPgain and pIgain. */
void STO_GetPLLGains(STO_PLL_Handle_t *pHandle, int16_t *pPgain, int16_t *pIgain);

/* Stores in the handler the new values for PLL gains. */
void STO_SetPLLGains(STO_PLL_Handle_t *pHandle, int16_t hPgain, int16_t hIgain);

/* Empty function. Could be declared to set instantaneous information on rotor mechanical angle. */
void STO_PLL_SetMecAngle(STO_PLL_Handle_t *pHandle, int16_t hMecAngle);

/* Sends locking info for PLL. */
void STO_SetPLL(STO_PLL_Handle_t *pHandle, int16_t hElSpeedDpp, int16_t hElAngle);

/* Exports estimated Bemf squared level stored in the handler. */
int32_t STO_PLL_GetEstimatedBemfLevel(STO_PLL_Handle_t *pHandle);

/* Exports observed Bemf squared level stored in the handler. */
int32_t STO_PLL_GetObservedBemfLevel(STO_PLL_Handle_t *pHandle);

/* Enables/Disables additional reliability check based on observed Bemf. */
void STO_PLL_BemfConsistencyCheckSwitch(STO_PLL_Handle_t *pHandle, bool bSel);

/* Checks if the Bemf is consistent. */
bool STO_PLL_IsBemfConsistent(STO_PLL_Handle_t *pHandle);

/* Checks the value of the variance. */
bool STO_PLL_IsVarianceTight(const STO_Handle_t *pHandle);

/* Forces the state-observer to declare convergency. */
void STO_PLL_ForceConvergency1(STO_Handle_t *pHandle);

/* Forces the state-observer to declare convergency. */
void STO_PLL_ForceConvergency2(STO_Handle_t *pHandle);

/* Sets the Absolute value of minimum mechanical speed required to validate the start-up. */
void STO_SetMinStartUpValidSpeedUnit(STO_PLL_Handle_t *pHandle, uint16_t hMinStartUpValidSpeed);

/* Sets the rotation direction in the handler. */
__weak void STO_SetDirection(STO_PLL_Handle_t *pHandle, int8_t direction);

/**
  * @}
  */

/**
  * @}
  */


#endif /*STO_PLL_SPEEDNPOSFDBK_H*/

/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
