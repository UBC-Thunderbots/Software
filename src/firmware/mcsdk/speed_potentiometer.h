/**
  ******************************************************************************
  * @file    speed_potentiometer.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the definitions and functions prototypes for the
  *          Speed Potentiometer component of the Motor Control SDK.
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
  * @ingroup SpeedPotentiometer
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SPEED_POTENTIOMETER_H
#define SPEED_POTENTIOMETER_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "potentiometer.h"
#include "mc_interface.h"


/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup Potentiometer
  * @{
  */

/** @addtogroup SpeedPotentiometer
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  Handle structure of a Speed Potentiometer component.
  * 
 *  A Speed Potentiometer component reads the value set on a potentiometer 
 * and uses it to set the rotation speed reference of a motor. To get potentiometer
 * values, an instance of a Potentiometer component is used.
 * 
  * This structure contains all the information needed for a Speed Potentiometer 
  * component instance to work.
  * 
  * See the @ref SpeedPotentiometer component documentation for more details.
 */
typedef struct 
{
  Potentiometer_Handle_t Pot;     /**< @brief Instance of the @ref Potentiometer component used
                                    *  to read the potentiometer 
                                    *
                                    * This structure must be set prior to calling POT_Init(). See 
                                    * the @ref Potentiometer component for all details.
                                    */

  MCI_Handle_t * pMCI;            /**< @brief Motor Control interface structure of the target motor
                                    * 
                                    *  This field must be set prior to calling POT_Init()
                                    */
  uint32_t RampSlope;             /**< @brief Acceleration to use when setting the speed reference  in #SPEED_UNIT/s.
                                    * 
                                    *  This field must be set prior to calling POT_Init()
                                    */
  uint16_t ConversionFactor;      /**< @brief Factor to convert speed between u16digit and #SPEED_UNIT.
                                    * 
                                    *  This field must be set prior to calling POT_Init()
                                    */
  uint16_t SpeedAdjustmentRange;  /**< @brief Represents the range used to trigger a speed ramp command. In u16digits 
                                    * 
                                    *  This field must be set prior to calling POT_Init()
                                    */
  uint16_t MinimumSpeed;          /**< @brief Minimum settable speed expressed in #SPEED_UNIT 
                                    * 
                                    *  This field must be set prior to calling POT_Init()
                                    */
  uint16_t LastSpeedRefSet;       /**< @brief Last speed reference set to the target motor. In u16digit */
  bool IsRunning;                 /**< @brief Used to track the transitions of the motor to and from the `RUN` state */
} SpeedPotentiometer_Handle_t;

/* Exported functions ------------------------------------------------------- */

/* Initializes a Speed Potentiometer component */
void SPDPOT_Init( SpeedPotentiometer_Handle_t * pHandle );
/* Clears the state of a Speed Potentiometer component */
void SPDPOT_Clear( SpeedPotentiometer_Handle_t * pHandle );
/* Reads the value of the potentiometer of a Speed Potentiometer component and sets
 * the rotation speed reference of the motor it targets acordingly  */
bool SPDPOT_Run( SpeedPotentiometer_Handle_t * pHandle );

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

#endif /* SPEED_POTENTIOMETER_H */
