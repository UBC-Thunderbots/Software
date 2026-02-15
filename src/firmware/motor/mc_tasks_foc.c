
/**
 ******************************************************************************
 * @file    mc_tasks_foc.c
 * @author  Motor Control SDK Team, ST Microelectronics
 * @brief   This file implements tasks definition
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */

#include "firmware/motor/mc_tasks.h"

/* Includes ------------------------------------------------------------------*/
// cstat -MISRAC2012-Rule-21.1
#include "firmware/motor/main.h"
// cstat +MISRAC2012-Rule-21.1
#include "firmware/motor/mc_app_hooks.h"
#include "firmware/motor/mc_interface.h"
#include "firmware/motor/mc_math.h"
#include "firmware/motor/mcsdk/digital_output.h"
#include "firmware/motor/mcsdk/mc_type.h"
#include "firmware/motor/mcsdk/pwm_common.h"
#include "firmware/motor/motorcontrol.h"
#include "firmware/motor/parameters_conversion.h"
#include "firmware/motor/regular_conversion_manager.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private define */
/* Private define ------------------------------------------------------------*/

/* USER CODE END Private define */

/* Private variables----------------------------------------------------------*/
OpenLoop_Handle_t *pOpenLoop[1] = {MC_NULL}; /* Only if M1 has OPEN LOOP */

static volatile uint16_t hBootCapDelayCounterM1   = ((uint16_t)0);
static volatile uint16_t hStopPermanencyCounterM1 = ((uint16_t)0);

#define M1_CHARGE_BOOT_CAP_TICKS (((uint16_t)SYS_TICK_FREQUENCY * (uint16_t)10) / 1000U)
#define M1_CHARGE_BOOT_CAP_DUTY_CYCLES                                                   \
    ((uint32_t)0.000 * ((uint32_t)PWM_PERIOD_CYCLES / 2U))
#define M2_CHARGE_BOOT_CAP_TICKS (((uint16_t)SYS_TICK_FREQUENCY * (uint16_t)10) / 1000U)
#define M2_CHARGE_BOOT_CAP_DUTY_CYCLES ((uint32_t)0 * ((uint32_t)PWM_PERIOD_CYCLES2 / 2U))

/* USER CODE BEGIN Private Variables */

/* USER CODE END Private Variables */

/* Private functions ---------------------------------------------------------*/
void TSK_MediumFrequencyTaskM1(void);
void FOC_InitAdditionalMethods(uint8_t bMotor);
void FOC_CalcCurrRef(uint8_t bMotor);
void TSK_MF_StopProcessing(uint8_t motor);

MCI_Handle_t *GetMCI(uint8_t bMotor);
static uint16_t FOC_CurrControllerM1(void);

void TSK_SafetyTask_PWMOFF(uint8_t motor);

/* USER CODE BEGIN Private Functions */

/* USER CODE END Private Functions */
/**
 * @brief  It initializes the whole MC core according to user defined
 *         parameters.
 */
__weak void FOC_Init(void)
{
    /* USER CODE BEGIN MCboot 0 */

    /* USER CODE END MCboot 0 */

    /**********************************************************/
    /*    PWM and current sensing component initialization    */
    /**********************************************************/
    pwmcHandle[M1] = &PWM_Handle_M1._Super;
    R1_Init(&PWM_Handle_M1);

    /* USER CODE BEGIN MCboot 1 */

    /* USER CODE END MCboot 1 */

    /******************************************************/
    /*   PID component initialization: speed regulation   */
    /******************************************************/
    PID_HandleInit(&PIDSpeedHandle_M1);

    /******************************************************/
    /*   Main speed sensor component initialization       */
    /******************************************************/
    HALL_Init(&HALL_M1);

    /******************************************************/
    /*   Speed & torque component initialization          */
    /******************************************************/
    STC_Init(pSTC[M1], &PIDSpeedHandle_M1, &HALL_M1._Super);

    /********************************************************/
    /*   PID component initialization: current regulation   */
    /********************************************************/
    PID_HandleInit(&PIDIqHandle_M1);
    PID_HandleInit(&PIDIdHandle_M1);

    /*************************************************/
    /*   Power measurement component initialization  */
    /*************************************************/
    pMPM[M1]->pVBS     = &(BusVoltageSensor_M1._Super);
    pMPM[M1]->pFOCVars = &FOCVars[M1];

    OL_Init(&OpenLoop_ParamsM1, &VirtualSpeedSensorM1); /* Only if M1 has open loop */
    pOpenLoop[M1] = &OpenLoop_ParamsM1;

    pREMNG[M1] = &RampExtMngrHFParamsM1;
    REMNG_Init(pREMNG[M1]);

    FOC_Clear(M1);
    FOCVars[M1].bDriveInput = EXTERNAL;
    FOCVars[M1].Iqdref      = STC_GetDefaultIqdref(pSTC[M1]);
    FOCVars[M1].UserIdref   = STC_GetDefaultIqdref(pSTC[M1]).d;
    MCI_SetSpeedMode(&Mci[M1]);

    MCI_ExecSpeedRamp(&Mci[M1], STC_GetMecSpeedRefUnitDefault(pSTC[M1]),
                      0); /* First command to STC */

    /* USER CODE BEGIN MCboot 2 */

    /* USER CODE END MCboot 2 */
}

/**
 * @brief Performs stop process and update the state machine.This function
 *        shall be called only during medium frequency task.
 */
void TSK_MF_StopProcessing(uint8_t motor)
{
    R1_SwitchOffPWM(pwmcHandle[motor]);

    FOC_Clear(motor);

    TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);
    Mci[motor].State = STOP;
}

/**
 * @brief Executes medium frequency periodic Motor Control tasks
 *
 * This function performs some of the control duties on Motor 1 according to the
 * present state of its state machine. In particular, duties requiring a periodic
 * execution at a medium frequency rate (such as the speed controller for instance)
 * are executed here.
 */
__weak void TSK_MediumFrequencyTaskM1(void)
{
    /* USER CODE BEGIN MediumFrequencyTask M1 0 */

    /* USER CODE END MediumFrequencyTask M1 0 */

    int16_t wAux = 0;
    MC_ControlMode_t mode;

    mode                 = MCI_GetControlMode(&Mci[M1]);
    bool IsSpeedReliable = HALL_CalcAvrgMecSpeedUnit(&HALL_M1, &wAux);
    PQD_CalcElMotorPower(pMPM[M1]);

    if (MCI_GetCurrentFaults(&Mci[M1]) == MC_NO_FAULTS)
    {
        if (MCI_GetOccurredFaults(&Mci[M1]) == MC_NO_FAULTS)
        {
            switch (Mci[M1].State)
            {
                case IDLE:
                {
                    if ((MCI_START == Mci[M1].DirectCommand) ||
                        (MCI_MEASURE_OFFSETS == Mci[M1].DirectCommand))
                    {
                        if (pwmcHandle[M1]->offsetCalibStatus == false)
                        {
                            (void)PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_START);
                            Mci[M1].State = OFFSET_CALIB;
                        }
                        else
                        {
                            /* Calibration already done. Enables only TIM channels */
                            pwmcHandle[M1]->OffCalibrWaitTimeCounter = 1u;
                            (void)PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_EXEC);
                            R1_TurnOnLowSides(pwmcHandle[M1],
                                              M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
                            TSK_SetChargeBootCapDelayM1(M1_CHARGE_BOOT_CAP_TICKS);
                            Mci[M1].State = CHARGE_BOOT_CAP;
                        }
                    }
                    else
                    {
                        /* Nothing to be done, FW stays in IDLE state */
                    }
                    break;
                }

                case OFFSET_CALIB:
                {
                    if (MCI_STOP == Mci[M1].DirectCommand)
                    {
                        TSK_MF_StopProcessing(M1);
                    }
                    else
                    {
                        if (PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_EXEC))
                        {
                            if (MCI_MEASURE_OFFSETS == Mci[M1].DirectCommand)
                            {
                                FOC_Clear(M1);
                                Mci[M1].DirectCommand = MCI_NO_COMMAND;
                                Mci[M1].State         = IDLE;
                            }
                            else
                            {
                                R1_TurnOnLowSides(pwmcHandle[M1],
                                                  M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
                                TSK_SetChargeBootCapDelayM1(M1_CHARGE_BOOT_CAP_TICKS);
                                Mci[M1].State = CHARGE_BOOT_CAP;
                            }
                        }
                        else
                        {
                            /* Nothing to be done, FW waits for offset calibration to
                             * finish */
                        }
                    }
                    break;
                }

                case CHARGE_BOOT_CAP:
                {
                    if (MCI_STOP == Mci[M1].DirectCommand)
                    {
                        TSK_MF_StopProcessing(M1);
                    }
                    else
                    {
                        if (TSK_ChargeBootCapDelayHasElapsedM1())
                        {
                            R1_SwitchOffPWM(pwmcHandle[M1]);

                            HALL_Clear(&HALL_M1);

                            FOC_Clear(M1);

                            if (mode == MCM_OPEN_LOOP_VOLTAGE_MODE ||
                                mode == MCM_OPEN_LOOP_CURRENT_MODE)
                            { /* In open loop mode, angle comes from virtual sensor */
                                STC_SetSpeedSensor(pSTC[M1],
                                                   &VirtualSpeedSensorM1._Super);
                            }
                            else
                            {
                                STC_SetSpeedSensor(pSTC[M1], &HALL_M1._Super);

                                FOC_InitAdditionalMethods(M1);
                                FOC_CalcCurrRef(M1);
                                STC_ForceSpeedReferenceToCurrentSpeed(
                                    pSTC[M1]); /* Init the reference speed to current
                                                  speed */
                            }
                            MCI_ExecBufferedCommands(
                                &Mci[M1]); /* Exec the speed ramp after changing of the
                                              speed sensor */
                            Mci[M1].State = RUN;
                            PWMC_SwitchOnPWM(pwmcHandle[M1]);
                        }
                        else
                        {
                            /* Nothing to be done, FW waits for bootstrap capacitor to
                             * charge */
                        }
                    }
                    break;
                }

                case RUN:
                {
                    if (MCI_STOP == Mci[M1].DirectCommand)
                    {
                        TSK_MF_StopProcessing(M1);
                    }
                    else
                    {
                        /* USER CODE BEGIN MediumFrequencyTask M1 2 */

                        /* USER CODE END MediumFrequencyTask M1 2 */

                        MCI_ExecBufferedCommands(&Mci[M1]);
                        if (mode != MCM_OPEN_LOOP_VOLTAGE_MODE &&
                            mode != MCM_OPEN_LOOP_CURRENT_MODE)
                        {
                            FOC_CalcCurrRef(M1);
                            if (!IsSpeedReliable)
                            {
                                MCI_FaultProcessing(&Mci[M1], MC_SPEED_FDBK, 0);
                            }
                            else
                            {
                                /* Nothing to do */
                            }
                        }
                        else
                        {
                            int16_t hForcedMecSpeedUnit;
                            /* Open Loop */
                            VSS_CalcAvrgMecSpeedUnit(&VirtualSpeedSensorM1,
                                                     &hForcedMecSpeedUnit);
                            OL_Calc(pOpenLoop[M1]);
                        }
                    }
                    break;
                }

                case STOP:
                {
                    if (TSK_StopPermanencyTimeHasElapsedM1())
                    {
                        /* USER CODE BEGIN MediumFrequencyTask M1 5 */

                        /* USER CODE END MediumFrequencyTask M1 5 */
                        Mci[M1].DirectCommand = MCI_NO_COMMAND;
                        Mci[M1].State         = IDLE;
                    }
                    else
                    {
                        /* Nothing to do, FW waits for to stop */
                    }
                    break;
                }

                case FAULT_OVER:
                {
                    if (MCI_ACK_FAULTS == Mci[M1].DirectCommand)
                    {
                        Mci[M1].DirectCommand = MCI_NO_COMMAND;
                        Mci[M1].State         = IDLE;
                    }
                    else
                    {
                        /* Nothing to do, FW stays in FAULT_OVER state until
                         * acknowledgement */
                    }
                    break;
                }

                case FAULT_NOW:
                {
                    Mci[M1].State = FAULT_OVER;
                    break;
                }

                default:
                    break;
            }
        }
        else
        {
            Mci[M1].State = FAULT_OVER;
        }
    }
    else
    {
        Mci[M1].State = FAULT_NOW;
    }
    /* USER CODE BEGIN MediumFrequencyTask M1 6 */

    /* USER CODE END MediumFrequencyTask M1 6 */
}

/**
 * @brief  It re-initializes the current and voltage variables. Moreover
 *         it clears qd currents PI controllers, voltage sensor and SpeednTorque
 *         controller. It must be called before each motor restart.
 *         It does not clear speed sensor.
 * @param  bMotor related motor it can be M1 or M2.
 */
__weak void FOC_Clear(uint8_t bMotor)
{
    /* USER CODE BEGIN FOC_Clear 0 */

    /* USER CODE END FOC_Clear 0 */
    MC_ControlMode_t mode;

    mode = MCI_GetControlMode(&Mci[bMotor]);

    ab_t NULL_ab               = {((int16_t)0), ((int16_t)0)};
    qd_t NULL_qd               = {((int16_t)0), ((int16_t)0)};
    alphabeta_t NULL_alphabeta = {((int16_t)0), ((int16_t)0)};

    FOCVars[bMotor].Iab        = NULL_ab;
    FOCVars[bMotor].Ialphabeta = NULL_alphabeta;
    FOCVars[bMotor].Iqd        = NULL_qd;
    if (mode != MCM_OPEN_LOOP_VOLTAGE_MODE && mode != MCM_OPEN_LOOP_CURRENT_MODE)
    {
        FOCVars[bMotor].Iqdref = NULL_qd;
    }
    else
    {
        /* Nothing to do */
    }
    FOCVars[bMotor].hTeref     = (int16_t)0;
    FOCVars[bMotor].Vqd        = NULL_qd;
    FOCVars[bMotor].Valphabeta = NULL_alphabeta;
    FOCVars[bMotor].hElAngle   = (int16_t)0;

    PID_SetIntegralTerm(pPIDIq[bMotor], ((int32_t)0));
    PID_SetIntegralTerm(pPIDId[bMotor], ((int32_t)0));

    STC_Clear(pSTC[bMotor]);

    PWMC_SwitchOffPWM(pwmcHandle[bMotor]);

    /* USER CODE BEGIN FOC_Clear 1 */

    /* USER CODE END FOC_Clear 1 */
}

/**
 * @brief  Use this method to initialize additional methods (if any) in
 *         START_TO_RUN state.
 * @param  bMotor related motor it can be M1 or M2.
 */
__weak void FOC_InitAdditionalMethods(uint8_t bMotor)  // cstat !RED-func-no-effect
{
    if (M_NONE == bMotor)
    {
        /* Nothing to do */
    }
    else
    {
        /* USER CODE BEGIN FOC_InitAdditionalMethods 0 */

        /* USER CODE END FOC_InitAdditionalMethods 0 */
    }
}

/**
 * @brief  It computes the new values of Iqdref (current references on qd
 *         reference frame) based on the required electrical torque information
 *         provided by oTSC object (internally clocked).
 *         If implemented in the derived class it executes flux weakening and/or
 *         MTPA algorithm(s). It must be called with the periodicity specified
 *         in oTSC parameters.
 * @param  bMotor related motor it can be M1 or M2.
 */
__weak void FOC_CalcCurrRef(uint8_t bMotor)
{
    qd_t IqdTmp;

    /* Enter critical section */
    /* Disable interrupts to avoid any interruption during Iqd reference latching */
    /* to avoid MF task writing them while HF task reading them */
    __disable_irq();
    IqdTmp = FOCVars[bMotor].Iqdref;

    /* Exit critical section */
    __enable_irq();

    /* USER CODE BEGIN FOC_CalcCurrRef 0 */

    /* USER CODE END FOC_CalcCurrRef 0 */
    MC_ControlMode_t mode;

    mode = MCI_GetControlMode(&Mci[bMotor]);
    if (INTERNAL == FOCVars[bMotor].bDriveInput &&
        (mode != MCM_OPEN_LOOP_VOLTAGE_MODE && mode != MCM_OPEN_LOOP_CURRENT_MODE))
    {
        FOCVars[bMotor].hTeref = STC_CalcTorqueReference(pSTC[bMotor]);
        IqdTmp.q               = FOCVars[bMotor].hTeref;
    }
    else
    {
        /* Nothing to do */
    }

    /* Enter critical section */
    /* Disable interrupts to avoid any interruption during Iqd reference restoring */
    __disable_irq();
    FOCVars[bMotor].Iqdref = IqdTmp;

    /* Exit critical section */
    __enable_irq();
    /* USER CODE BEGIN FOC_CalcCurrRef 1 */

    /* USER CODE END FOC_CalcCurrRef 1 */
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif

/**
 * @brief  Executes the Motor Control duties that require a high frequency rate and a
 * precise timing.
 *
 *  This is mainly the FOC current control loop. It is executed depending on the state of
 * the Motor Control subsystem (see the state machine(s)).
 *
 * @retval Number of the  motor instance which FOC loop was executed.
 */
__weak uint8_t FOC_HighFrequencyTask(uint8_t bMotorNbr)
{
    uint16_t hFOCreturn;
    /* USER CODE BEGIN HighFrequencyTask 0 */

    /* USER CODE END HighFrequencyTask 0 */

    (void)HALL_CalcElAngle(&HALL_M1);

    /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_1 */

    /* USER CODE END HighFrequencyTask SINGLEDRIVE_1 */
    hFOCreturn = FOC_CurrControllerM1();
    /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_2 */

    /* USER CODE END HighFrequencyTask SINGLEDRIVE_2 */
    if (hFOCreturn == MC_DURATION)
    {
        MCI_FaultProcessing(&Mci[M1], MC_DURATION, 0);
    }
    else
    {
        if (RUN == Mci[M1].State)
        {
            int16_t hObsAngle = SPD_GetElAngle(&HALL_M1._Super);
            (void)VSS_CalcElAngle(&VirtualSpeedSensorM1, &hObsAngle);
        }
        else
        {
            /* Nothing to do */
        }
        /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_3 */

        /* USER CODE END HighFrequencyTask SINGLEDRIVE_3 */
    }

    return (bMotorNbr);
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
 * @brief It executes the core of FOC drive that is the controllers for Iqd
 *        currents regulation. Reference frame transformations are carried out
 *        accordingly to the active speed sensor. It must be called periodically
 *        when new motor currents have been converted
 * @param this related object of class CFOC.
 * @retval int16_t It returns MC_NO_FAULTS if the FOC has been ended before
 *         next PWM Update event, MC_DURATION otherwise
 */
inline uint16_t FOC_CurrControllerM1(void)
{
    qd_t Iqd, Vqd;
    ab_t Iab;
    alphabeta_t Ialphabeta, Valphabeta;
    int16_t hElAngle;
    uint16_t hCodeError;
    SpeednPosFdbk_Handle_t *speedHandle;
    MC_ControlMode_t mode;

    mode        = MCI_GetControlMode(&Mci[M1]);
    speedHandle = STC_GetSpeedSensor(pSTC[M1]);
    hElAngle    = SPD_GetElAngle(speedHandle);
    PWMC_GetPhaseCurrents(pwmcHandle[M1], &Iab);
    RCM_ExecNextConv();
    Ialphabeta = MCM_Clarke(Iab);
    Iqd        = MCM_Park(Ialphabeta, hElAngle);
    Vqd.q      = PI_Controller(pPIDIq[M1], (int32_t)(FOCVars[M1].Iqdref.q) - Iqd.q);
    Vqd.d      = PI_Controller(pPIDId[M1], (int32_t)(FOCVars[M1].Iqdref.d) - Iqd.d);
    if (mode == MCM_OPEN_LOOP_VOLTAGE_MODE)
    {
        Vqd = OL_VqdConditioning(pOpenLoop[M1]);
    }
    else
    {
        /* Nothing to do */
    }
    Vqd = Circle_Limitation(&CircleLimitationM1, Vqd);
    hElAngle += SPD_GetInstElSpeedDpp(speedHandle) * REV_PARK_ANGLE_COMPENSATION_FACTOR;
    Valphabeta = MCM_Rev_Park(Vqd, hElAngle);
    RCM_ReadOngoingConv();
    hCodeError = PWMC_SetPhaseVoltage(pwmcHandle[M1], Valphabeta);
    PWMC_CalcPhaseCurrentsEst(pwmcHandle[M1], Iqd, hElAngle);

    FOCVars[M1].Vqd        = Vqd;
    FOCVars[M1].Iab        = Iab;
    FOCVars[M1].Ialphabeta = Ialphabeta;
    FOCVars[M1].Iqd        = Iqd;
    FOCVars[M1].Valphabeta = Valphabeta;
    FOCVars[M1].hElAngle   = hElAngle;

    return (hCodeError);
}

/* USER CODE BEGIN mc_task 0 */

/* USER CODE END mc_task 0 */

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
