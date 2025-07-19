/**
  ******************************************************************************
  * @file   speed_potentiometer.c
  * @author Motor Control SDK Team, ST Microelectronics
  * @brief  This file provides firmware functions that implement the features
  *         of the Speed Potentiometer component of the Motor Control SDK.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 20222 STMicroelectronics.
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

/* Includes ------------------------------------------------------------------*/
#include "speed_potentiometer.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup Potentiometer
 * @{
 */

/** @defgroup SpeedPotentiometer Speed Potentiometer
  * @brief Potentiometer reading component that sets the motor 
  *         speed reference from the potentiometer value
  * 
  *  The Speed Potentiometer component uses the services of the 
  * @ref Potentiometer component to read a potentiometer. Values read
  * from this potentiometer are used to set the rotation speed reference
  * of a motor.
  * 
  *  The state of a Speed Potentiometer component is maintained in a 
  * SpeedPotentiometer_Handle_t structure. To use the Speed Potentiometer component, 
  * a SpeedPotentiometer_Handle_t structure needs to be instanciated and initialized. 
  * The initialization is performed thanks to the SPDPOT_Init() function. Prior to 
  * calling this function, some of the fields of this structure need to be given a 
  * value. See the Potentiometer_Handle_t and below for more details on this.
  * 
  *  A Speed Potentiometer component sets speed references by executing speed ramps 
  * thanks to the STC_ExecRamp() function. Prior to doing so, the component checks 
  * if the motor is started (that is: if its state machine has reached the #RUN state) 
  * and if its control modality is [Speed](@ref MCM_SPEED_MODE). If either of these
  * conditions is not met, no speed ramp is executed. 
  * 
  *  In addition, a speed ramp is executed only if the value of the potentiometer at 
  * that time differs from the one of the previous ramp that the component filed by 
  * a configurable amount. This amount is gieven by the 
  * @ref SpeedPotentiometer_Handle_t::SpeedAdjustmentRange "SpeedAdjustmentRange" field
  * of the Handle structure.
  * 
  *  The speed range accessible through the potentiometer is bounded by a minimum speed 
  * and a maximum speed. the minimum speed is stated at compile time in the 
  * @ref SpeedPotentiometer_Handle_t::MinimumSpeed "MinimumSpeed" field of the Handle
  * structure. The maximum speed is deduced from the 
  * @ref SpeedPotentiometer_Handle_t::ConversionFactor "ConversionFactor" field thanks to 
  * the following formula: 
  * 
  * $$
  *   MaximumSpeed = \frac{(65536 - MinimumSpeed \times ConversionFactor )}{ConversionFactor}
  * $$
  * 
  * where 65536 is the maximum value that the potentiometer (the ADC actually) can produce.
  *  
  *  The @ref SpeedPotentiometer_Handle_t::MinimumSpeed "MinimumSpeed" can be set so that when
  * the potentiometer is set to its minimum value, the motor stil spins. This is useful when 
  * the Motor Control application uses a sensorless speed and position sensing algorithm that
  * cannot work below a given speed.
  * 
  *  The duration of speed ramps is controlled with the @ref SpeedPotentiometer_Handle_t::RampSlope "RampSlope"
  * field of the Handle. This field actually sets the acceleration used to change from one speed
  * to another.
  * 
  *  A potentiometer provides absolute values. A Speed Potentiometer component turns these 
  * values into either positive or negative speed references depending on the actual speed
  * of the motor. So, if a motor is started with a negative speed, the references set by the
  * Speed Potentiometer component targetting it will also be negative.
  * 
  *  Values measured from the potentiometer are expressed in "u16digits": these are 16-bit values directly 
  * read from an ADC. They need to be converted to the [speed unit](#SPEED_UNIT) used by the API 
  * of the motor control library in order to generatethe speed references for the ramps.
  *
  * @note In the current version of the Speed Potentiometer component, the periodic ADC
  * measures **must** be performed on the Medium Frequency Task. This can be done by using
  * the MC_APP_PostMediumFrequencyHook_M1() function of the @ref MCAppHooks service
  * for instance.
  * 
  * @{
  */

/* Public functions ----------------------------------------------------------*/

/**
 * @brief Initializes a Speed Potentiometer component
 * 
 * This function must be called once before the component can be used.
 * 
 * @param pHandle Handle on the Speed Potentiometer component to initialize
 */
void SPDPOT_Init( SpeedPotentiometer_Handle_t * pHandle )
{
#ifdef NULL_PTR_SPD_POT
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    POT_Init( & pHandle->Pot );

    SPDPOT_Clear( pHandle );
#ifdef NULL_PTR_SPD_POT
  }
#endif /* NULL_PTR_SPD_POT */
}

/**
 * @brief Clears the state of a Speed Potentiometer component
 * 
 * The state of the @ref Potentiometer component instance used by the 
 * Speed Potentiometer component is also cleared.
 * 
 * @param pHandle Handle on the Speed Potentiometer component
 */
void SPDPOT_Clear( SpeedPotentiometer_Handle_t * pHandle )
{
#ifdef NULL_PTR_SPD_POT
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    POT_Clear( (Potentiometer_Handle_t *) pHandle );

    pHandle->LastSpeedRefSet = (uint16_t) 0;
    pHandle->IsRunning = false;
#ifdef NULL_PTR_SPD_POT
  }
#endif /* NULL_PTR_SPD_POT */
}

/**
 * @brief Reads the value of the potentiometer of a Speed Potentiometer component and sets
 *        the rotation speed reference of the motor it targets acordingly
 * 
 *  The potentiometer handled by the Speed Potentiometer component is read. If its value
 * differs from the one that was last used to set the speed reference of the motor by more
 * than @ref SpeedPotentiometer_Handle_t::SpeedAdjustmentRange "SpeedAdjustmentRange", then
 * this new value is used to set a new speed reference.
 * 
 * If the motor does not spin (it is not in the #RUN state) or if the current motor control 
 * modality is not speed (#MCM_SPEED_MODE), nothing is done.
 * 
 *  This function needs to be called periodically. See the @ref Potentiometer 
 * documentation for more details.

 * @param pHandle Handle on the Speed Potentiometer component
 */
bool SPDPOT_Run( SpeedPotentiometer_Handle_t * pHandle )
{
    bool retVal = false;
#ifdef NULL_PTR_SPD_POT
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    SpeednTorqCtrl_Handle_t * pSTC = pHandle->pMCI->pSTC;

    POT_TakeMeasurement( (Potentiometer_Handle_t *) pHandle );

    if ( MCM_SPEED_MODE == STC_GetControlMode( pSTC ) )
    {
      if ( RUN == MCI_GetSTMState( pHandle->pMCI ) ) 
      {
        if ( false == pHandle->IsRunning ) 
        {
            /* Making sure the potentiometer value is going to be considered. */            
            pHandle->LastSpeedRefSet ^= 65535;

            /* Remember that the motor is running */
            pHandle->IsRunning = true;
        } else { /* Nothing to do */ }

        if ( POT_ValidValueAvailable( (Potentiometer_Handle_t *) pHandle ) ) {
            uint16_t potValue = POT_GetValue( (Potentiometer_Handle_t *) pHandle );

            if ( potValue <= pHandle->LastSpeedRefSet - pHandle->SpeedAdjustmentRange ||
                potValue >= pHandle->LastSpeedRefSet + pHandle->SpeedAdjustmentRange )
            {
                SpeednPosFdbk_Handle_t *speedHandle;
                uint32_t rampDuration;
                int16_t currentSpeed;
                int16_t requestedSpeed;
                int16_t deltaSpeed;

                speedHandle = STC_GetSpeedSensor( pSTC );
                currentSpeed = SPD_GetAvrgMecSpeedUnit( speedHandle );
                requestedSpeed = (int16_t)( potValue / pHandle->ConversionFactor + pHandle->MinimumSpeed );

                deltaSpeed = (int16_t) requestedSpeed - ( (currentSpeed >=0) ? currentSpeed : - currentSpeed );
                if ( deltaSpeed < 0 ) deltaSpeed = - deltaSpeed;

                rampDuration = (uint32_t) deltaSpeed * 1000U / pHandle->RampSlope;

                if ( currentSpeed < 0 ) requestedSpeed = - requestedSpeed;

                STC_ExecRamp( pSTC, requestedSpeed, rampDuration );

                pHandle->LastSpeedRefSet = potValue;
                retVal = true;
            }
        } else { /* Nothing to do */ }
      }
      else
      {
        //
        pHandle->IsRunning = false;
      }
    }
    else 
    {
        pHandle->IsRunning = false;
    }
#ifdef NULL_PTR_SPD_POT
  }
#endif /* NULL_PTR_SPD_POT */

    return retVal;
}