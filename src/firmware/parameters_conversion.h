
/**
  ******************************************************************************
  * @file    parameters_conversion.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file includes the proper Parameter conversion on the base
  *          of stdlib for the first drive
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
#ifndef PARAMETERS_CONVERSION_H
#define PARAMETERS_CONVERSION_H

#include "mc_math.h"
#include "parameters_conversion_f0xx.h"
#include "pmsm_motor_parameters.h"
#include "drive_parameters.h"
#include "power_stage_parameters.h"

#define ADC_REFERENCE_VOLTAGE  3.3

/************************* CONTROL FREQUENCIES & DELAIES **********************/
#define TF_REGULATION_RATE 	(uint32_t) ((uint32_t)(PWM_FREQUENCY)/(REGULATION_EXECUTION_RATE))

/* TF_REGULATION_RATE_SCALED is TF_REGULATION_RATE divided by PWM_FREQ_SCALING to allow more dynamic */
#define TF_REGULATION_RATE_SCALED (uint16_t) ((uint32_t)(PWM_FREQUENCY)/(REGULATION_EXECUTION_RATE*PWM_FREQ_SCALING))

/* DPP_CONV_FACTOR is introduce to compute the right DPP with TF_REGULATOR_SCALED  */
#define DPP_CONV_FACTOR (65536/PWM_FREQ_SCALING)

/* Current conversion from Ampere unit to 16Bit Digit */
#define CURRENT_CONV_FACTOR		(uint16_t)((65536.0 * RSHUNT * AMPLIFICATION_GAIN)/ADC_REFERENCE_VOLTAGE)
#define CURRENT_CONV_FACTOR_INV		(1.0 / ((65536.0 * RSHUNT * AMPLIFICATION_GAIN)/ADC_REFERENCE_VOLTAGE))

#define REP_COUNTER 			(uint16_t) ((REGULATION_EXECUTION_RATE *2u)-1u)
#define SYS_TICK_FREQUENCY          ( uint16_t )2000U
#define UI_TASK_FREQUENCY_HZ        10U

#define MEDIUM_FREQUENCY_TASK_RATE        (uint16_t)SPEED_LOOP_FREQUENCY_HZ
#define MF_TASK_OCCURENCE_TICKS           (SYS_TICK_FREQUENCY/SPEED_LOOP_FREQUENCY_HZ)-1u

#define UI_TASK_OCCURENCE_TICKS           (SYS_TICK_FREQUENCY/UI_TASK_FREQUENCY_HZ)-1u
#define SERIALCOM_TIMEOUT_OCCURENCE_TICKS (SYS_TICK_FREQUENCY/SERIAL_COM_TIMEOUT_INVERSE)-1u
#define SERIALCOM_ATR_TIME_TICKS          (uint16_t)(((SYS_TICK_FREQUENCY * SERIAL_COM_ATR_TIME_MS) / 1000u) - 1u)

#define MAX_APPLICATION_SPEED_UNIT ((MAX_APPLICATION_SPEED_RPM*SPEED_UNIT)/U_RPM)
#define MIN_APPLICATION_SPEED_UNIT ((MIN_APPLICATION_SPEED_RPM*SPEED_UNIT)/U_RPM)

#define MAX_APPLICATION_SPEED_UNIT2 ((MAX_APPLICATION_SPEED_RPM2*SPEED_UNIT)/_RPM)
#define MIN_APPLICATION_SPEED_UNIT2 ((MIN_APPLICATION_SPEED_RPM2*SPEED_UNIT)/_RPM)

/**************************   VOLTAGE CONVERSIONS  Motor 1 *************************/
#define OVERVOLTAGE_THRESHOLD_d   (uint16_t)(OV_VOLTAGE_THRESHOLD_V*65535/\
                                  (ADC_REFERENCE_VOLTAGE/VBUS_PARTITIONING_FACTOR))
#define OVERVOLTAGE_THRESHOLD_LOW_d   (uint16_t)(OV_VOLTAGE_THRESHOLD_V)*65535/\
                                  (ADC_REFERENCE_VOLTAGE/VBUS_PARTITIONING_FACTOR))
#define UNDERVOLTAGE_THRESHOLD_d  (uint16_t)((UD_VOLTAGE_THRESHOLD_V*65535)/\
                                  ((uint16_t)(ADC_REFERENCE_VOLTAGE/\
                                                           VBUS_PARTITIONING_FACTOR)))
#define INT_SUPPLY_VOLTAGE          (uint16_t)(65536/ADC_REFERENCE_VOLTAGE)

#define DELTA_TEMP_THRESHOLD        (OV_TEMPERATURE_THRESHOLD_C- T0_C)
#define DELTA_V_THRESHOLD           (dV_dT * DELTA_TEMP_THRESHOLD)
#define OV_TEMPERATURE_THRESHOLD_d  ((V0_V + DELTA_V_THRESHOLD)*INT_SUPPLY_VOLTAGE)

#define DELTA_TEMP_HYSTERESIS        (OV_TEMPERATURE_HYSTERESIS_C)
#define DELTA_V_HYSTERESIS           (dV_dT * DELTA_TEMP_HYSTERESIS)
#define OV_TEMPERATURE_HYSTERESIS_d  (DELTA_V_HYSTERESIS*INT_SUPPLY_VOLTAGE)

#define ALIGNMENT_ANGLE_S16      (int16_t)  (ALIGNMENT_ANGLE_DEG*65536u/360u)

/*************** Timer for PWM generation & currenst sensing parameters  ******/
#define PWM_PERIOD_CYCLES (uint16_t)((ADV_TIM_CLK_MHz*(uint32_t)1000000u/((uint32_t)(PWM_FREQUENCY)))& ( uint16_t )0xFFFE)

#define DEADTIME_NS  SW_DEADTIME_NS

#define DEAD_TIME_ADV_TIM_CLK_MHz (ADV_TIM_CLK_MHz * TIM_CLOCK_DIVIDER)
#define DEAD_TIME_COUNTS_1  (DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/1000uL)

#if (DEAD_TIME_COUNTS_1 <= 255)
#define DEAD_TIME_COUNTS (uint16_t) DEAD_TIME_COUNTS_1
#elif (DEAD_TIME_COUNTS_1 <= 508)
#define DEAD_TIME_COUNTS (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/2) /1000uL) + 128)
#elif (DEAD_TIME_COUNTS_1 <= 1008)
#define DEAD_TIME_COUNTS (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/8) /1000uL) + 320)
#elif (DEAD_TIME_COUNTS_1 <= 2015)
#define DEAD_TIME_COUNTS (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/16) /1000uL) + 384)
#else
#define DEAD_TIME_COUNTS 510
#endif

#define DTCOMPCNT (uint16_t)((DEADTIME_NS * ADV_TIM_CLK_MHz) / 2000)
#define TON_NS  500
#define TOFF_NS 500
#define TON  (uint16_t)((TON_NS * ADV_TIM_CLK_MHz)  / 2000)
#define TOFF (uint16_t)((TOFF_NS * ADV_TIM_CLK_MHz) / 2000)
/**********************/
/* MOTOR 1 ADC Timing */
/**********************/
#define SAMPLING_TIME ((ADC_SAMPLING_CYCLES * ADV_TIM_CLK_MHz) / ADC_CLK_MHz) /* In ADV_TIMER CLK cycles*/
#define TRISE ((TRISE_NS * ADV_TIM_CLK_MHz)/1000uL)
#define TDEAD ((uint16_t)((DEADTIME_NS * ADV_TIM_CLK_MHz)/1000))
#define TNOISE ((uint16_t)((TNOISE_NS*ADV_TIM_CLK_MHz)/1000))
#define TMAX_TNTR ((uint16_t)((MAX_TNTR_NS * ADV_TIM_CLK_MHz)/1000uL))
#define TAFTER ((uint16_t)(TDEAD + TMAX_TNTR))
#define TBEFORE ((uint16_t)(((ADC_TRIG_CONV_LATENCY_CYCLES + ADC_SAMPLING_CYCLES) * ADV_TIM_CLK_MHz) / ADC_CLK_MHz)  + 1U)
#define TMIN ((uint16_t)( TAFTER + TBEFORE ))
#define HTMIN ((uint16_t)(TMIN >> 1))
#define CHTMIN ((uint16_t)(TMIN/(REGULATION_EXECUTION_RATE*2)))
#if (TRISE > SAMPLING_TIME)
#define MAX_TRTS (2 * TRISE)
#else
#define MAX_TRTS (2 * SAMPLING_TIME)
#endif

/* USER CODE BEGIN temperature */

#define M1_VIRTUAL_HEAT_SINK_TEMPERATURE_VALUE   25u
#define M1_TEMP_SW_FILTER_BW_FACTOR      250u

/* USER CODE END temperature */

#define PQD_CONVERSION_FACTOR (float)((( 1.732 * ADC_REFERENCE_VOLTAGE ) /\
             ( RSHUNT * AMPLIFICATION_GAIN ))/65536.0f)

/****** Prepares the UI configurations according the MCconfxx settings ********/

#define DAC_ENABLE
#define DAC_OP_ENABLE

/* Motor 1 settings */
#define FW_ENABLE

#define DIFFTERM_ENABLE

/* Sensors setting */

#define MAIN_SCFG UI_SCODE_ENC

#define AUX_SCFG 0x0

#define PLLTUNING_ENABLE

#define UI_CFGOPT_PFC_ENABLE

/*******************************************************************************
  * UI configurations settings. It can be manually overwritten if special
  * configuartion is required.
*******************************************************************************/

/* Specific options of UI */
#define UI_CONFIG_M1 ( UI_CFGOPT_NONE DAC_OP_ENABLE FW_ENABLE DIFFTERM_ENABLE \
  | (MAIN_SCFG << MAIN_SCFG_POS) | (AUX_SCFG << AUX_SCFG_POS) | UI_CFGOPT_SETIDINSPDMODE PLLTUNING_ENABLE UI_CFGOPT_PFC_ENABLE | UI_CFGOPT_PLLTUNING)

#define UI_CONFIG_M2

#define DIN_ACTIVE_LOW Bit_RESET
#define DIN_ACTIVE_HIGH Bit_SET

#define DOUT_ACTIVE_HIGH   DOutputActiveHigh
#define DOUT_ACTIVE_LOW    DOutputActiveLow

/**********  AUXILIARY ENCODER TIMER MOTOR 1 *************/
#define M1_PULSE_NBR ( (4 * (M1_ENCODER_PPR)) - 1 )
#define M1_ENC_IC_FILTER  8
#define SPD_TIM_M1_IRQHandler TIM2_IRQHandler

#define LPF_FILT_CONST ((int16_t)(32767 * 0.5))
/* MMI Table Motor 1 MAX_MODULATION_100_PER_CENT */
#define MAX_MODULE 32767

#define SAMPLING_CYCLE_CORRECTION 0.5 /* Add half cycle required by STM32F031C6Tx ADC */
#define LL_ADC_SAMPLINGTIME_1CYCLES_5 LL_ADC_SAMPLINGTIME_1CYCLE_5

#define LL_ADC_SAMPLING_CYCLE(CYCLE) LL_ADC_SAMPLINGTIME_ ## CYCLE ## CYCLES_5 //cstat !MISRAC2012-Rule-20.10 !DEFINE-hash-multiple

#endif /*PARAMETERS_CONVERSION_H*/

/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
