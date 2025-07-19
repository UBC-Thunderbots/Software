/*
  ******************************************************************************
  * @file    speed_ctrl.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the following features
  *          of the Speed Control component of the Motor Control SDK.
  *
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

/* Includes ------------------------------------------------------------------*/
#include "speed_ctrl.h"
#include "speed_pos_fdbk.h"

#include "mc_type.h"

#define CHECK_BOUNDARY

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup SpeednTorqCtrl Speed Control
  * @brief Speed Control component of the Motor Control SDK
  *
  * @todo Document the Speed Control "module".
  *
  * @{
  */

/**
  * @brief  Initializes all the object variables, usually it has to be called
  *         once right after object creation.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @param  pPI the PI object used as controller for the speed regulation.
  * @param  SPD_Handle the speed sensor used to perform the speed regulation.
  * @retval none.
  */
__weak void STC_Init( SpeednTorqCtrl_Handle_t * pHandle, PID_Handle_t * pPI, SpeednPosFdbk_Handle_t * SPD_Handle )
{

  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    pHandle->PISpeed = pPI;
    pHandle->SPD = SPD_Handle;
    pHandle->Mode = pHandle->ModeDefault;
    pHandle->SpeedRefUnitExt = ((int32_t)pHandle->MecSpeedRefUnitDefault) * 65536;
    pHandle->DutyCycleRef = ((uint32_t)pHandle->DutyCycleRefDefault) * 65536;
    pHandle->TargetFinal = 0;
    pHandle->RampRemainingStep = 0U;
    pHandle->IncDecAmount = 0;
  }
}

/**
  * @brief It sets in real time the speed sensor utilized by the STC.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @param SPD_Handle Speed sensor component to be set.
  * @retval none
  */
__weak void STC_SetSpeedSensor( SpeednTorqCtrl_Handle_t * pHandle, SpeednPosFdbk_Handle_t * SPD_Handle )
{
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    pHandle->SPD = SPD_Handle;
  }
}

/**
  * @brief It returns the speed sensor utilized by the FOC.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval SpeednPosFdbk_Handle_t speed sensor utilized by the FOC.
  */
__weak SpeednPosFdbk_Handle_t * STC_GetSpeedSensor( SpeednTorqCtrl_Handle_t * pHandle )
{
  return ((MC_NULL ==  pHandle) ? MC_NULL : pHandle->SPD);
}

/**
  * @brief  It should be called before each motor restart. If STC is set in
            speed mode, this method resets the integral term of speed regulator.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval none.
  */
__weak void STC_Clear( SpeednTorqCtrl_Handle_t * pHandle )
{
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    if (MCM_SPEED_MODE == pHandle->Mode)
    {
      PID_SetIntegralTerm(pHandle->PISpeed, 0);
    }
    pHandle->DutyCycleRef = ((uint32_t)pHandle->DutyCycleRefDefault) * 65536;	
  }
}

/**
  * @brief  Get the current mechanical rotor speed reference.
  *         Expressed in the unit defined by SPEED_UNIT.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval int16_t current mechanical rotor speed reference.
  *         Expressed in the unit defined by SPEED_UNIT.
  */
__weak int16_t STC_GetMecSpeedRefUnit( SpeednTorqCtrl_Handle_t * pHandle )
{
#ifdef NO_FULL_MISRA_C_COMPLIANCY_SPD_TORQ_CTRL
  //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
  return ((MC_NULL == pHandle) ? 0 : (int16_t)(pHandle->SpeedRefUnitExt >> 16));
#else
  return ((MC_NULL == pHandle) ? 0 : (int16_t)(pHandle->SpeedRefUnitExt / 65536));
#endif
}

/**
  * @brief  Get the current motor dutycycle reference. This value represents
  *         actually the dutycycle reference expressed in digit.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval uint16_t current dutycycle reference. This value is actually expressed in digit.
  */
__weak uint16_t STC_GetDutyCycleRef(SpeednTorqCtrl_Handle_t *pHandle)
{
#ifdef NO_FULL_MISRA_C_COMPLIANCY_SPD_TORQ_CTRL
  //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
  return ((MC_NULL == pHandle) ? 0 : (uint16_t)(pHandle->DutyCycleRef >> 16));
#else
  return ((MC_NULL == pHandle) ? 0 : (uint16_t)(pHandle->DutyCycleRef / 65536));
#endif
}

/**
  * @brief  Set the modality of the speed controller. Two modality
  *         are available Torque mode and Speed mode.
  *         In Torque mode is possible to set directly the dutycycle 
  *         reference or execute a motor dutycycle ramp. This value represents
  *         actually the reference expressed in digit.
  *         In Speed mode is possible to set the mechanical rotor speed
  *         reference or execute a speed ramp. The required motor dutycycle is
  *         automatically calculated by the STC.
  *         This command interrupts the execution of any previous ramp command
  *         maintaining the last value of dutycycle.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @param  bMode modality of STC. It can be one of these two settings:
  *         MCM_TORQUE_MODE to enable the Torque mode or MCM_SPEED_MODE to
  *         enable the Speed mode.
  * @retval none
  */
__weak void STC_SetControlMode( SpeednTorqCtrl_Handle_t * pHandle, MC_ControlMode_t bMode )
{
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    pHandle->Mode = bMode;
    pHandle->RampRemainingStep = 0u; /* Interrupts previous ramp. */
  }
}

/**
  * @brief  Get the modality of the speed controller.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval MC_ControlMode_t It returns the modality of STC. It can be one of
  *         these two values: MCM_TORQUE_MODE or MCM_SPEED_MODE.
  */
__weak MC_ControlMode_t STC_GetControlMode( SpeednTorqCtrl_Handle_t * pHandle )
{
  return ((MC_NULL == pHandle) ? MCM_TORQUE_MODE : pHandle->Mode);
}

/**
  * @brief  Starts the execution of a ramp using new target and duration. This
  *         command interrupts the execution of any previous ramp command.
  *         The generated ramp will be in the modality previously set by
  *         STC_SetControlMode method.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @param  hTargetFinal final value of command. This is different accordingly
  *         the STC modality.
  *         hTargetFinal is the value of mechanical rotor speed reference at the end 
  *         of the ramp.Expressed in the unit defined by SPEED_UNIT
  * @param  hDurationms the duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the value.
  * @retval bool It return false if the absolute value of hTargetFinal is out of
  *         the boundary of the application (Above max application speed or below min  
  *         application speed in this case the command is ignored and the
  *         previous ramp is not interrupted, otherwise it returns true.
  */
__weak bool STC_ExecRamp(SpeednTorqCtrl_Handle_t *pHandle, int16_t hTargetFinal, uint32_t hDurationms)
{
  bool allowedRange = true;

  if (MC_NULL == pHandle)
  {
    allowedRange = false;
  }
  else
  {
    uint32_t wAux;
    int32_t wAux1;
    int16_t hCurrentReference;

    /* Check if the hTargetFinal is out of the bound of application. */
    if (MCM_TORQUE_MODE == pHandle->Mode)
    {
      hCurrentReference = STC_GetDutyCycleRef(pHandle);
#ifdef CHECK_BOUNDARY
      if ((int32_t)hTargetFinal > (int32_t)pHandle->MaxPositiveDutyCycle)
      {
        allowedRange = false;
      }
#endif
    }
    else
    {
#ifdef NO_FULL_MISRA_C_COMPLIANCY_SPD_TORQ_CTRL
      //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
      hCurrentReference = (int16_t)(pHandle->SpeedRefUnitExt >> 16);
#else
      hCurrentReference = (int16_t)(pHandle->SpeedRefUnitExt / 65536);
#endif

#ifdef CHECK_BOUNDARY
      if ((int32_t)hTargetFinal > (int32_t)pHandle->MaxAppPositiveMecSpeedUnit)
      {
        allowedRange = false;
      }
      else if (hTargetFinal < pHandle->MinAppNegativeMecSpeedUnit)
      {
        allowedRange = false;
      }
      else if ((int32_t)hTargetFinal < (int32_t)pHandle->MinAppPositiveMecSpeedUnit)
      {
        if (hTargetFinal > pHandle->MaxAppNegativeMecSpeedUnit)
        {
          allowedRange = false;
        }
      }
      else
      {
        /* Nothing to do */
      }
#endif
    }

    if (true == allowedRange)
    {
      /* Interrupts the execution of any previous ramp command */
      if (0U == hDurationms)
      {
        if (MCM_SPEED_MODE == pHandle->Mode)
        {
          pHandle->SpeedRefUnitExt = ((int32_t)hTargetFinal) * 65536;
        }
        else
        {
          pHandle->DutyCycleRef = ((int32_t)hTargetFinal) * 65536;
        }
        pHandle->RampRemainingStep = 0U;
        pHandle->IncDecAmount = 0;
      }
      else
      {
        /* Store the hTargetFinal to be applied in the last step */
        pHandle->TargetFinal = hTargetFinal;

        /* Compute the (wRampRemainingStep) number of steps remaining to complete
        the ramp. */
        wAux = ((uint32_t)hDurationms) * ((uint32_t)pHandle->STCFrequencyHz);
        wAux /= 1000U;
        pHandle->RampRemainingStep = wAux;
        pHandle->RampRemainingStep++;

        /* Compute the increment/decrement amount (wIncDecAmount) to be applied to
        the reference value at each CalcSpeedReference. */
        wAux1 = (((int32_t)hTargetFinal) - ((int32_t)hCurrentReference)) * 65536;
        wAux1 /= ((int32_t)pHandle->RampRemainingStep);
        pHandle->IncDecAmount = wAux1;
      }
    }
  }
  return (allowedRange);
}

/**
  * @brief  This command interrupts the execution of any previous ramp command.
  *         The last value of mechanical rotor speed reference is maintained.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval none
  */
__weak void STC_StopRamp(SpeednTorqCtrl_Handle_t *pHandle)
{
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    pHandle->RampRemainingStep = 0U;
    pHandle->IncDecAmount = 0;
  }
}

/**
  * @brief  It is used to compute the new value of motor speed reference. It
  *         must be called at fixed time equal to hSTCFrequencyHz. It is called
  *         passing as parameter the speed sensor used to perform the speed
  *         regulation.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval int16_t motor dutycycle reference. This value represents actually the
  *         dutycycle expressed in digit.
  */
__weak uint16_t STC_CalcSpeedReference(SpeednTorqCtrl_Handle_t *pHandle)
{
  uint16_t hDutyCycleReference;

  if (MC_NULL == pHandle)
  {
    hDutyCycleReference = 0;
  }
  else
  {
    int32_t wCurrentReference;
    int16_t hMeasuredSpeed;
    int16_t hTargetSpeed;
    int16_t hError;

    if (MCM_TORQUE_MODE == pHandle->Mode)
    {
      wCurrentReference = pHandle->DutyCycleRef;
    }
    else
    {
      wCurrentReference = pHandle->SpeedRefUnitExt;
    }

    /* Update the speed reference or the torque reference according to the mode
       and terminates the ramp if needed. */
    if (pHandle->RampRemainingStep > 1U)
    {
      /* Increment/decrement the reference value. */
      wCurrentReference += pHandle->IncDecAmount;

      /* Decrement the number of remaining steps */
      pHandle->RampRemainingStep--;
    }
    else if (1U == pHandle->RampRemainingStep)
    {
      /* Set the backup value of hTargetFinal. */
      wCurrentReference = ((int32_t)pHandle->TargetFinal) * 65536;
      pHandle->RampRemainingStep = 0U;
    }
    else
    {
      /* Do nothing. */
    }

    if (MCM_SPEED_MODE == pHandle->Mode)
    {
      /* Run the speed control loop */

      /* Compute speed error */
#ifdef NO_FULL_MISRA_C_COMPLIANCY_SPD_TORQ_CTRL
      //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
      hTargetSpeed = (int16_t)(wCurrentReference >> 16);
#else
      hTargetSpeed = (int16_t)(wCurrentReference / 65536);
#endif
      hMeasuredSpeed = SPD_GetAvrgMecSpeedUnit(pHandle->SPD);
      if (hTargetSpeed < 0) hError = hMeasuredSpeed - hTargetSpeed;
      else hError = hTargetSpeed - hMeasuredSpeed;
      hDutyCycleReference = PI_Controller(pHandle->PISpeed, (int32_t)hError);

      pHandle->SpeedRefUnitExt = wCurrentReference;
      pHandle->DutyCycleRef = ((int32_t)hDutyCycleReference) * 65536;
    }
    else
    {
      pHandle->DutyCycleRef = wCurrentReference;
#ifdef NO_FULL_MISRA_C_COMPLIANCY_SPD_TORQ_CTRL
      //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
      hDutyCycleReference = (int16_t)(wCurrentReference >> 16);
#else
      hDutyCycleReference = (int16_t)(wCurrentReference / 65536);
#endif
    }
  }
  return (hDutyCycleReference);
}

/**
  * @brief  Get the Default mechanical rotor speed reference.
  *         Expressed in the unit defined by #SPEED_UNIT.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval int16_t It returns the Default mechanical rotor speed. 
  *         Expressed in the unit defined by #SPEED_UNIT
  */
__weak int16_t STC_GetMecSpeedRefUnitDefault(SpeednTorqCtrl_Handle_t *pHandle)
{
  return ((MC_NULL == pHandle) ? 0 : pHandle->MecSpeedRefUnitDefault);
}

/**
  * @brief  Returns the Application maximum positive value of rotor speed. Expressed in the unit defined by #SPEED_UNIT.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  */
__weak uint16_t STC_GetMaxAppPositiveMecSpeedUnit(SpeednTorqCtrl_Handle_t *pHandle)
{
  return ((MC_NULL == pHandle) ? 0U : pHandle->MaxAppPositiveMecSpeedUnit);
}

/**
  * @brief  Returns the Application minimum negative value of rotor speed. Expressed in the unit defined by #SPEED_UNIT.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  */
__weak int16_t STC_GetMinAppNegativeMecSpeedUnit(SpeednTorqCtrl_Handle_t *pHandle)
{
  return ((MC_NULL == pHandle) ? 0 : pHandle->MinAppNegativeMecSpeedUnit);
}

/**
  * @brief  Check if the settled speed ramp has been completed.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval bool It returns true if the ramp is completed, false otherwise.
  */
__weak bool STC_RampCompleted(SpeednTorqCtrl_Handle_t *pHandle)
{
  bool retVal = false;

  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    if (0U == pHandle->RampRemainingStep)
    {
      retVal = true;
    }
  }
  return (retVal);
}

/**
  * @brief  Stop the execution of speed ramp.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval bool It returns true if the command is executed, false otherwise.
  */
__weak bool STC_StopSpeedRamp(SpeednTorqCtrl_Handle_t *pHandle)
{
  bool retVal = false;

  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    if (MCM_SPEED_MODE == pHandle->Mode)
    {
      pHandle->RampRemainingStep = 0u;
      retVal = true;
    }
  }
  return (retVal);
}

/**
  * @brief  Force the speed reference to the curren speed. It is used
  *         at the START_RUN state to initialize the speed reference.
  * @param  pHandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval none
  */
__weak void STC_ForceSpeedReferenceToCurrentSpeed(SpeednTorqCtrl_Handle_t *pHandle)
{
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    pHandle->SpeedRefUnitExt = ((int32_t)SPD_GetAvrgMecSpeedUnit(pHandle->SPD)) * (int32_t)65536;
  }
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
