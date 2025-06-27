/**
  ******************************************************************************
  * @file    mc_type.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Motor Control SDK global types definitions
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
  * @ingroup MC_Type
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MC_TYPE_H
#define MC_TYPE_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup MC_Type Motor Control types
  * @{
  */

/**
  * @define MISRA_C_2004_BUILD
  * @brief Used to build the library with MISRA C support
  *
  * Uncomment #define MISRA_C_2004_BUILD to build the library including
  * "stm32fxxx_MisraCompliance.h" instead of "stm32fxxx.h".
  *
  * This will build the library in 'strict ISO/ANSI C' and in
  * compliance with MISRA C 2004 rules (check project options).
  *
  * @note Do not use this flag with the current version of the SDK.
  */
/*#define MISRA_C_2004_BUILD*/

#include <mc_stm_types.h>

/* char definition to match Misra Dir 4.6 typedefs that indicate size and
 * signedness should be used in place ofthe basic numerical types */
typedef int8_t          char_t;
typedef uint8_t         uchar_t;

#ifndef _MATH
typedef float           float_t;
#endif


/** @name Macros to use bit banding capability */
/** @{ */
#define BB_REG_BIT_SET(regAddr,bit_number) *(uint32_t *) (0x42000000+(((uint32_t)regAddr - 0x40000000 )<<5) + (bit_number <<2)) = (uint32_t)(0x1u)
#define BB_REG_BIT_CLR(regAddr,bit_number) (*(uint32_t *) (0x42000000+(((uint32_t)regAddr - 0x40000000)<<5) + (bit_number <<2)) = (uint32_t)(0x0u))
#define BB_REG_BIT_READ(regAddr,bit_number) (*(uint32_t *) (0x42000000+(((uint32_t)regAddr - 0x40000000)<<5) + (bit_number <<2)) )
/** @} */

/** @brief Not initialized pointer */
#define MC_NULL    (void *)(0x0)

/** @name Motor identification macros */
/** @{ */
#define M1      (uint8_t)(0x0)  /*!< Motor 1.*/
#define M2      (uint8_t)(0x1)  /*!< Motor 2.*/
#define M_NONE  (uint8_t)(0xFF) /*!< None motor.*/
/** @} */

/** 
 * @anchor fault_codes 
 * @name Fault codes
 * The symbols below define the codes associated to the faults that the 
 * Motor Control subsystem can raise.
 * @{ */
#define  MC_NO_ERROR     ((uint16_t)0x0000) /**< @brief No error.*/
#define  MC_NO_FAULTS    ((uint16_t)0x0000) /**< @brief No error.*/
#define  MC_DURATION     ((uint16_t)0x0001) /**< @brief Error: FOC rate to high.*/
#define  MC_OVER_VOLT    ((uint16_t)0x0002) /**< @brief Error: Software over voltage.*/
#define  MC_UNDER_VOLT   ((uint16_t)0x0004) /**< @brief Error: Software under voltage.*/
#define  MC_OVER_TEMP    ((uint16_t)0x0008) /**< @brief Error: Software over temperature.*/
#define  MC_START_UP     ((uint16_t)0x0010) /**< @brief Error: Startup failed.*/
#define  MC_SPEED_FDBK   ((uint16_t)0x0020) /**< @brief Error: Speed feedback.*/
#define  MC_BREAK_IN     ((uint16_t)0x0040) /**< @brief Error: Emergency input (Over current).*/
#define  MC_SW_ERROR     ((uint16_t)0x0080) /**< @brief Software Error.*/
/** @}*/

/** @name Dual motor Frequency comparison definition */
/** @{ */
#define SAME_FREQ   0U
#define HIGHER_FREQ 1U
#define LOWER_FREQ  2U

#define HIGHEST_FREQ 1U
#define LOWEST_FREQ  2U
/** @} */

/**
  * @brief union type definition for u32 to Float conversion and vice versa
  */
//cstat -MISRAC2012-Rule-19.2
typedef union _FLOAT_U32_
{
  uint32_t  U32_Val;
  float   Float_Val;
} FloatToU32;
//cstat +MISRAC2012-Rule-19.2

/**
  * @brief Two components q, d type definition
  */
typedef struct
{
  int16_t q;
  int16_t d;
} qd_t;

/**
  * @brief Two components q, d in float type
  */

typedef struct
{
    float q;
    float d;
} qd_f_t;

/**
  * @brief Two components a,b type definition
  */
typedef struct
{
  int16_t a;
  int16_t b;
} ab_t;

/**
  * @brief Two components a,b in float type
  */
typedef struct
{
  float a;
  float b;
} ab_f_t;

/**
  * @brief Two components alpha, beta type definition
  */
typedef struct
{
  int16_t alpha;
  int16_t beta;
} alphabeta_t;

/* ACIM definitions start */
typedef struct 
{
  float fS_Component1;
  float fS_Component2;
} Signal_Components;

/** 
  * @brief  Two components type definition 
  */
typedef struct 
{
  int16_t qVec_Component1;
  int16_t qVec_Component2;
} Vector_s16_Components;
/* ACIM definitions end */

/**
  * @brief  ADConv_t type definition, it is used by PWMC_ADC_SetSamplingTime method of PWMnCurrFdbk class for user defined A/D regular conversions
  */
typedef struct
{
  uint8_t Channel;   /*!< Integer channel number, from 0 to 15 */
  uint8_t SamplTime; /*!< Sampling time selection, ADC_SampleTime_nCycles5*/
} ADConv_t;

/**
  * @brief  SensorType_t type definition, it's used in BusVoltageSensor and TemperatureSensor component parameters structures
  *       to specify whether the sensor is real or emulated by SW
  */
typedef enum
{
  REAL_SENSOR, VIRTUAL_SENSOR
} SensorType_t;

/**
  * @brief  DOutputState_t type definition, it's used by DOUT_SetOutputState method of DigitalOutput class to specify the
  *     required output state
  */
typedef enum
{
  INACTIVE, ACTIVE
} DOutputState_t;

/**
  * @brief  DrivingMode_t type definition, it's used by 
  *         Bemf_ADC class to specify the driving mode type
  */
typedef enum
{
  VM, /**< @brief Voltage mode.*/
  CM   /**< @brief Current mode.*/
} DrivingMode_t;

/**
  * @brief  Specifies the control modality of the motor
  */
typedef enum
{
  MCM_OBSERVING_MODE = 0,       /**< @brief not used */
  MCM_OPEN_LOOP_VOLTAGE_MODE,   /**< @brief Open loop, duty cycle set as reference */
  MCM_OPEN_LOOP_CURRENT_MODE,   /**< @brief Open loop, q & d currents set as reference */
  MCM_SPEED_MODE,               /**< @brief Closed loop, Speed mode.*/
  MCM_TORQUE_MODE,              /**< @brief Closed loop, Torque mode.*/
  MCM_PROFILING_MODE,           /**< @brief not used */
  MCM_SHORTED_MODE,             /**< @brief not used */
  MCM_POSITION_MODE,            /**< @brief Closed loop, sensored position control mode */
  MCM_MODE_NUM                  /**< @brief Number of modes in enum */
} MC_ControlMode_t;



/**
  * @brief Structure type definition for feed-forward constants tuning
  */
typedef struct
{
  int32_t wConst_1D;
  int32_t wConst_1Q;
  int32_t wConst_2;
} FF_TuningStruct_t;

/**
  * @brief structure type definition for phase offsets setting/getting. In case of single shunt
  *        only phaseAOffset is relevant.
  */
typedef struct
{
  int32_t phaseAOffset;
  int32_t phaseBOffset;
  int32_t phaseCOffset;
} PolarizationOffsets_t;

/**
  * @brief  Current references source type, internal or external to FOCDriveClass
  */
typedef enum
{
  INTERNAL, EXTERNAL
} CurrRefSource_t ;

/**
  * @brief  FOC variables structure
  */
typedef struct
{  //cstat !MISRAC2012-Dir-4.8

  ab_t Iab;                     /**< @brief Stator current on stator reference frame abc */
  alphabeta_t Ialphabeta;       /**< @brief Stator current on stator reference frame alfa-beta*/
  qd_t IqdHF;                   /**< @brief Stator current on stator reference frame alfa-beta*/
  qd_t Iqd;                     /**< @brief Stator current on rotor reference frame qd */
  qd_t Iqdref;                  /**< @brief Stator current on rotor reference frame qd */
  int16_t UserIdref;            /**< @brief User value for the Idref stator current */
  qd_t Vqd;                     /**< @brief Phase voltage on rotor reference frame qd */
  alphabeta_t Valphabeta;       /**< @brief Phase voltage on stator reference frame alpha-beta*/
  int16_t hTeref;               /**< @brief Reference torque */
  int16_t hElAngle;             /**< @brief Electrical angle used for reference frame transformation  */
  uint16_t hCodeError;          /**< @brief Error Code */
  CurrRefSource_t bDriveInput;  /**< @brief Specifies whether the current reference source must be
                                  *         #INTERNAL or #EXTERNAL*/
} FOCVars_t, *pFOCVars_t;

/**
  * @brief  6step variables structure
  */
typedef struct
{
  uint16_t DutyCycleRef;              /**< @brief Reference speed */
  uint16_t hCodeError;         /**< @brief error message */
  CurrRefSource_t bDriveInput; /**< @brief It specifies whether the current reference source must be
                                 *         #INTERNAL or #EXTERNAL*/
  int16_t qElAngle;
} SixStepVars_t, *pSixStepVars_t;

/**
  * @brief  Low side or enabling signal definition
  */

typedef enum
{
  LS_DISABLED  = 0x0U,    /**< @brief Low side signals and enabling signals always off.
                                         It is equivalent to DISABLED. */
  LS_PWM_TIMER = 0x1U,  /**< @brief Low side PWM signals are generated by timer. It is
                                         equivalent to ENABLED. */
  ES_GPIO   = 0x2U             /**< @brief Enabling signals are managed by GPIOs (L6230 mode).*/
} LowSideOutputsFunction_t;

/** @name UserInterface related exported definitions */
/** @{ */
#define OPT_NONE    0x00 /**< @brief No UI option selected. */
#define OPT_COM     0x02 /**< @brief Bit field indicating that the UI uses serial communication. */
#define OPT_DAC     0x04 /**< @brief Bit field indicating that the UI uses real DAC. */
#define OPT_DACT    0x08 /**< @brief Bit field indicating that the UI uses RC Timer DAC. */
#define OPT_DACS    0x10 /**< @brief Bit field indicating that the UI uses SPI communication. */
#define OPT_DACF3   0x40 /**< @brief Bit field indicating that the UI uses DAC for STM32F3. */
#define OPT_DACF072 0x80 /**< @brief Bit field indicating that the UI uses DAC for STM32F072. */
/** @} */

#define MAIN_SCFG_POS (28)
#define AUX_SCFG_POS  (24)

#define MAIN_SCFG_VALUE(x) (((x)>>MAIN_SCFG_POS)& ( uint8_t )0x0F)
#define AUX_SCFG_VALUE(x)  (((x)>>AUX_SCFG_POS)& ( uint8_t )0x0F)

/** @name PFC related exported definitions */
/** @{ */

#define PFC_SWE             0x0001U /**< @brief PFC Software error. */
#define PFC_HW_PROT         0x0002U /**< @brief PFC hardware protection. */
#define PFC_SW_OVER_VOLT    0x0004U /**< @brief PFC software over voltage. */
#define PFC_SW_OVER_CURRENT 0x0008U /**< @brief PFC software over current. */
#define PFC_SW_MAINS_FREQ   0x0010U /**< @brief PFC mains frequency error. */
#define PFC_SW_MAIN_VOLT    0x0020U /**< @brief PFC mains voltage error. */
/** @} */

/** @name Definitions exported for the DAC channel used as reference for protection */
/** @{ */
#define AO_DISABLED 0x00U /**< @brief Analog output disabled.*/
#define AO_DEBUG    0x01U /**< @brief Analog output debug.*/
#define VREF_OCPM1  0x02U /**< @brief Voltage reference for over current protection of motor 1.*/
#define VREF_OCPM2  0x03U /**< @brief Voltage reference for over current protection of motor 2.*/
#define VREF_OCPM12 0x04U /**< @brief Voltage reference for over current protection of both motors.*/
#define VREF_OVPM12 0x05U /**< @brief Voltage reference for over voltage protection of both motors.*/
/** @} */

/** @name ADC channel number definitions */
/** @{ */
#define MC_ADC_CHANNEL_0     0
#define MC_ADC_CHANNEL_1     1
#define MC_ADC_CHANNEL_2     2
#define MC_ADC_CHANNEL_3     3
#define MC_ADC_CHANNEL_4     4
#define MC_ADC_CHANNEL_5     5
#define MC_ADC_CHANNEL_6     6
#define MC_ADC_CHANNEL_7     7
#define MC_ADC_CHANNEL_8     8
#define MC_ADC_CHANNEL_9     9
#define MC_ADC_CHANNEL_10    10
#define MC_ADC_CHANNEL_11    11
#define MC_ADC_CHANNEL_12    12
#define MC_ADC_CHANNEL_13    13
#define MC_ADC_CHANNEL_14    14
#define MC_ADC_CHANNEL_15    15
#define MC_ADC_CHANNEL_16    16
#define MC_ADC_CHANNEL_17    17
#define MC_ADC_CHANNEL_18    18
/** @} */

/** @name Utility macros definitions */
/** @{ */
#define RPM2MEC01HZ(rpm) (int16_t)((int32_t)(rpm)/6)
#define MAX(a,b) (((a)>(b))?(a):(b))
/** @} */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* MC_TYPE_H */
/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
