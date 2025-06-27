/**
  ******************************************************************************
  * @file    potentiometer.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the definitions and functions prototypes for the
  *          Potentiometer component of the Motor Control SDK.
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
  * @ingroup Potentiometer
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "regular_conversion_manager.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup Potentiometer
  * @{
  */

/* Exported types ------------------------------------------------------------*/

 /**
  * @brief  Handle structure of a Potentiometer component.
  *
  * A potentiometer component aims at measuring the voltage 
  * across a potentiometer thanks to an ADC. 
  * 
  * This structure contains all the information needed for a Potentiometer 
  * component instance to work.
  * 
  * See the @ref Potentiometer component documentation for more details.
  */
typedef struct
{
  RegConv_t PotRegConv;             /**< @brief Structure configuring the 
                                      *  [Regular Conversion Manager](#RCM) to 
                                      *  measure of the level of the potentiometer
                                      * 
                                      * The fields of this structure must be set prior
                                      * to calling the POT_Init() function
                                      */
  uint16_t  * PotMeasArray;         /**< @brief Array for storing raw potentiometer measures
                                      *
                                      *  This field must point to a reserved array of uint16_t
                                      * prior to calling the POT_Init() function. The size of
                                      * this array must be @f$2^{LPFilterBandwidthPOW2}@f$.
                                      * 
                                      * See #LPFilterBandwidthPOW2.
                                      */
  uint32_t  PotValueAccumulator;    /**< @brief accumulated potentiometer measures */
  uint8_t   LPFilterBandwidthPOW2;  /**< @brief Number of measures used to compute the average
                                      *         value, expressed as a power of 2
                                      *
                                      *  For instance, if the number of measures is 16, the
                                      * value of this field must be 4 since @f$16 = 2^4@f$.
                                      * 
                                      *  This field must be set prior to calling POT_Init()
                                      */
  uint8_t   LPFilterBandwidth;      /**< @brief Number of measures used to compute the average value
                                      *
                                      * This value is set to @f$2^{LPFilterBandwidthPOW2}@f$ by 
                                      * the POT_Init() function.
                                      * 
                                      * See #LPFilterBandwidthPOW2.
                                      */
  uint8_t   Index;                  /**< @brief position where the next measure will be put in #PotMeqsArray */
  uint8_t   PotRegConvHandle;       /**< @brief Handle on the [Regular Conversion Manager](#RCM)
                                      *  to manage the measures of the potentiometer */
  bool      Valid;                  /**< @brief Indicates the validity of #PotValueAccumulator
                                      *
                                      * See the @ref Potentiometer documentation for more details.
                                      */
} Potentiometer_Handle_t;

/* Exported functions ------------------------------------------------------- */

/* Potentiometer component initialization */
void POT_Init( Potentiometer_Handle_t * pHandle );
/* Clears the state of a Potentiometer component */
void POT_Clear( Potentiometer_Handle_t * pHandle );
/* Measures the voltage of the potentiometer */
void POT_TakeMeasurement( Potentiometer_Handle_t * pHandle );

/* Returns the current potentiometer value */
uint16_t POT_GetValue( Potentiometer_Handle_t * pHandle );
/* Returns true if the current potentiometer value is valid */
bool POT_ValidValueAvailable( Potentiometer_Handle_t * pHandle);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* POTENTIOMETER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
