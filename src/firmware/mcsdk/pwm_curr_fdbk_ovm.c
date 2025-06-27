/**
  ******************************************************************************
  * @file    pwm_curr_fdbk_ovm.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the following features
  *          of the PWM & Current Feedback component of the Motor Control SDK:
  *
  *           * current sensing
  *           * regular ADC conversion execution
  *           * space vector modulation
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
  * @ingroup pwm_curr_fdbk
  */

/* Includes ------------------------------------------------------------------*/
#include "pwm_curr_fdbk.h"
#include "mc_math.h"
#include "mc_type.h"



/* Private defines ------------------------------------------------------------*/

/* @brief The float number 1.0 equal to 32768 ; represents the vector from center to an angle of the hexagon*/
#define    OVM_ONE_POINT_ZERO     ((int32_t)32768)
#define    OVM_GAIN_ARRAY_SIZE    ((int32_t)192)
#define    OVM_GAMMA_ARRAY_SIZE   ((int32_t)100)
#define    OVM_GAMMA_ARRAY_OFFSET ((int32_t)92)
#define    OVM_VREF_MODE1_START   ((int32_t)29717)
#define    OVM_VREF_MODE2_START   ((int32_t)31186)
#define    OVM_VREF_MODE2_END     ((int32_t)32768)
#define    OVM_VREF_INDEX_STEP    ((int32_t)16)
#define    OVM_1_DIV_SQRT3        ((int32_t)18919)
#define    OVM_1_DIV_PI           ((int32_t)10430)
#define    OVM_PI_DIV_6           ((int32_t)17157)
#define    OVM_3_DIV_PI           ((int32_t)31291)
#define    OVM_SQRT3              ((int32_t)56754)

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk 
  * 
  * @{
  */

/* Private variables ----------------------------------------------------------*/

/**
 * @brief Vector modules T1 and T2. Specified in @ref overmodulation.md 
 */
typedef struct
{
  int32_t T1;
  int32_t T2;
} Vector_Time_t;

/**
 * @brief Overmodulation modes. Specified in @ref overmodulation.md
 */
typedef enum
{
  OVM_LINEAR = 0,   /**< Linear mode. */
  OVM_1 = 1,        /**< Overmodulation mode 1. */
  OVM_2 = 2,        /**< Overmodulation mode 2. */
  OVM_ERROR = 3     /**< Error output. */
} OVM_Mode_t;

/**
  * @}
  */

/**
  * @}
  */
 
/* Private function prototypes -----------------------------------------------*/
static Vector_Time_t PWMC_RecalcT1T2_OVM(Vector_Time_t time, OVM_Mode_t mode, int16_t gamma );


#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
 /*
  * @brief Recalculates vectors T1 and T2 stored in @p time structure,
  *        depending on the overmodulation @p mode and @p gamma gain. 
  * 
  * @retval New values of T1 & T2.
  */
static Vector_Time_t PWMC_RecalcT1T2_OVM(Vector_Time_t time, OVM_Mode_t mode, int16_t gamma )
{
  int32_t sum_t1_t2, offset, gain, t1_temp;
  Vector_Time_t time_prime;

  time_prime.T1 = 0;
  time_prime.T2 = 0;

  if ((OVM_LINEAR == mode) || (OVM_1 == mode))
  {
    sum_t1_t2 = time.T1 + time.T2;

    if (sum_t1_t2 > OVM_ONE_POINT_ZERO)
    {
      time_prime.T1 = ((time.T1 * OVM_ONE_POINT_ZERO) / sum_t1_t2);
      time_prime.T2 = OVM_ONE_POINT_ZERO - time_prime.T1;
    }
    else
    {
      time_prime.T1 = time.T1;
      time_prime.T2 = time.T2;
    }
  }
  else if (OVM_2 == mode)
  {
    if (time.T1 > OVM_ONE_POINT_ZERO)
    {
      time_prime.T1 = OVM_ONE_POINT_ZERO;
      time_prime.T2 = 0;
    }
    else if (time.T2 > OVM_ONE_POINT_ZERO)
    {
      time_prime.T1 = 0;
      time_prime.T2 = OVM_ONE_POINT_ZERO;
    }
    else
    {
      offset = (OVM_3_DIV_PI * gamma) / OVM_ONE_POINT_ZERO;
      gain = (OVM_PI_DIV_6 * OVM_ONE_POINT_ZERO) / (OVM_PI_DIV_6 - gamma);

      sum_t1_t2 = time.T1 + time.T2;
      sum_t1_t2 = ((0 == sum_t1_t2) ? 1 : sum_t1_t2);
      t1_temp = (time.T1 * OVM_ONE_POINT_ZERO) / sum_t1_t2;
      t1_temp = t1_temp - offset;
      if (t1_temp < 0)
      {
        t1_temp = 0;
      }
      else
      {
        /* Nothing to do */
      }
      if (gain > OVM_ONE_POINT_ZERO)
      {
        gain = gain / OVM_ONE_POINT_ZERO;
        time_prime.T1 = t1_temp * gain;
      }
      else
      {
        time_prime.T1 = (t1_temp * gain) / OVM_ONE_POINT_ZERO;
      }
      if (time_prime.T1 > OVM_ONE_POINT_ZERO)
      {
        time_prime.T1 = OVM_ONE_POINT_ZERO;
      }
      else
      {
        /* Nothing to do */
      }
      time_prime.T2 = OVM_ONE_POINT_ZERO - time_prime.T1;
    }
  }
  else  /* error mode output 0 to protect */
  {
    time_prime.T1 = 0;
    time_prime.T2 = 0;
  }
  return (time_prime);
}

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk 
  * 
  * @{
  */

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
  * @brief  Converts input voltage components @f$ V_{\alpha} @f$ and @f$ V_{\beta} @f$ into duty cycles
  *         and feeds them to the inverter with overmodulation function.
  * 
  * @param  pHandle: Handler of the current instance of the PWM component.
  * @param  Valfa_beta: Voltage Components expressed in the @f$(\alpha, \beta)@f$ reference frame.
  * @retval #MC_NO_ERROR if no error occurred or #MC_DURATION if the duty cycles were
  *         set too late for being taken into account in the next PWM cycle.
  */
uint16_t PWMC_SetPhaseVoltage_OVM(PWMC_Handle_t *pHandle, alphabeta_t Valfa_beta)
{
  /* Private variables ----------------------------------------------------------*/
  /* overmodulation gain array */
  static const uint16_t aOVMGain[OVM_GAIN_ARRAY_SIZE]={\
  31291,31291,31293,31295,31298,31300,31302,31306,31309,31314,\
  31319,31322,31328,31334,31338,31344,31350,31357,31364,31371,\
  31379,31386,31394,31402,31410,31419,31427,31439,31448,31457,\
  31470,31479,31492,31502,31515,31526,31539,31554,31568,31579,\
  31594,31609,31624,31639,31655,31675,31691,31707,31728,31745,\
  31766,31783,31805,31827,31845,31868,31891,31914,31942,31966,\
  31990,32019,32044,32074,32104,32134,32165,32202,32233,32271,\
  32303,32341,32386,32425,32470,32516,32562,32609,32662,32716,\
  32777,32838,32907,32982,33059,33144,33236,33343,33466,33612,\
  33797,34106,34463,34507,34551,34596,34640,34684,34729,34779,\
  34824,34869,34920,34971,35017,35068,35120,35178,35230,35282,\
  35340,35392,35451,35509,35568,35627,35686,35752,35811,35877,\
  35943,36009,36075,36148,36214,36287,36360,36434,36507,36581,\
  36661,36742,36822,36903,36990,37078,37159,37253,37342,37436,\
  37531,37627,37729,37831,37933,38042,38152,38261,38378,38495,\
  38612,38736,38860,38991,39122,39261,39399,39545,39691,39844,\
  40004,40165,40332,40507,40682,40871,41061,41264,41469,41680,\
  41906,42139,42387,42649,42911,43188,43488,43801,44137,44487,\
  44866,45275,45713,46195,46715,47300,47958,48720,49629,50759,\
  52346,56660,\
  };

  /* overmodulation gamma array */
  static const int16_t aOVMGamma[OVM_GAMMA_ARRAY_SIZE]={\
  52,154,255,354,453,551,648,757,852,947,\
  1052,1157,1249,1352,1454,1566,1666,1765,1875,1972,\
  2079,2186,2291,2395,2499,2612,2713,2824,2934,3042,\
  3150,3266,3372,3486,3599,3711,3821,3931,4049,4166,\
  4281,4395,4517,4637,4748,4875,4992,5115,5238,5359,\
  5487,5614,5739,5870,6000,6129,6263,6396,6528,6665,\
  6800,6941,7080,7224,7367,7514,7659,7809,7963,8115,\
  8272,8432,8590,8757,8922,9096,9268,9442,9624,9809,\
  10001,10200,10395,10597,10810,11028,11255,11487,11731,11987,\
  12254,12539,12835,13158,13507,13895,14335,14853,15530,17125,\
  };

  uint16_t retvalue;
#ifdef NULL_PTR_PWM_CUR_FDB_OVM
  if (NULL == pHandle)
  {
    retvalue = 0U;
  }
  else
  {
#endif
    int16_t index;
    int32_t wX, wY, wZ, wUAlpha, wUBeta;
    int32_t vref, gain, neg_beta_cmd_div_sqrt3, duty_a, duty_b, duty_c ;
    int32_t gama = 0;

    OVM_Mode_t ovm_mode_flag;
    Vector_Time_t vector_time;

    /*transfer vref to vcmd*/
    vref = (int32_t)MCM_Modulus( Valfa_beta.alpha, Valfa_beta.beta );

    if (vref < OVM_VREF_MODE1_START)      /*linear range*/
    {
      wUAlpha = Valfa_beta.alpha;
      wUBeta = Valfa_beta.beta;
      ovm_mode_flag = OVM_LINEAR;
    }
    else if (vref < OVM_VREF_MODE2_START) /*OVM mode 1 range*/
    {
      index = (int16_t)((vref - OVM_VREF_MODE1_START) / OVM_VREF_INDEX_STEP);
      gain = (int32_t)aOVMGain[index];
      wUAlpha = (Valfa_beta.alpha * gain) / OVM_ONE_POINT_ZERO;
      wUBeta = (Valfa_beta.beta * gain) / OVM_ONE_POINT_ZERO;
      ovm_mode_flag = OVM_1;
    }
    else if (vref < OVM_VREF_MODE2_END)  /*OVM mode 2 range*/
    {
      index = (int16_t)((vref - OVM_VREF_MODE1_START) / OVM_VREF_INDEX_STEP);
      gain = (int32_t)aOVMGain[index];
      wUAlpha = (Valfa_beta.alpha * gain) / OVM_ONE_POINT_ZERO;
      wUBeta = (Valfa_beta.beta * gain) / OVM_ONE_POINT_ZERO;
      if (index > OVM_GAMMA_ARRAY_OFFSET)
      {
        gama = aOVMGamma[index - OVM_GAMMA_ARRAY_OFFSET];
      }
      else
      {
        gama = aOVMGamma[0];
      }
      ovm_mode_flag = OVM_2;
    }
    else                                 /*out of OVM mode 2 range only a protection to prevent the code to a undefined branch*/
    {
      wUAlpha = 0;
      wUBeta = 0;
      ovm_mode_flag = OVM_ERROR;
    }

    /*Vcmd  to X, Y, Z*/
    neg_beta_cmd_div_sqrt3 = ((-wUBeta) * OVM_1_DIV_SQRT3) / OVM_ONE_POINT_ZERO;
    wX = neg_beta_cmd_div_sqrt3 * 2;           /* x=-(2/sqrt(3))*beta */
    wY = wUAlpha + neg_beta_cmd_div_sqrt3;     /* x=alpha-(1/sqrt(3))*beta */
    wZ = -wUAlpha + neg_beta_cmd_div_sqrt3;;   /* x=-alpha-(1/sqrt(3))*beta */

    /* Sector calculation from wX, wY, wZ */
    if (wY < 0)
    {
      if (wZ < 0)
      {
        pHandle->Sector = SECTOR_5;
        vector_time.T1 = -wY;
        vector_time.T2 = -wZ;
        vector_time = PWMC_RecalcT1T2_OVM( vector_time, ovm_mode_flag, (int16_t)gama );
        duty_a = 16384 + ((-vector_time.T1 + vector_time.T2) / 2);
#ifndef FULL_MISRA_C_COMPLIANCY_PW_CURR_FDB_OVM
        //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
        duty_a = (((int32_t)pHandle->PWMperiod) * duty_a) >> 16;
        duty_b = 16384 + ((-vector_time.T1 - vector_time.T2) / 2);
        //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
        duty_b = (((int32_t)pHandle->PWMperiod) * duty_b) >> 16;
        duty_c = 16384 + ((vector_time.T1 + vector_time.T2) / 2);
        //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
        duty_c = (((int32_t)pHandle->PWMperiod) * duty_c) >> 16;
#else
        duty_a = (((int32_t)pHandle->PWMperiod) * duty_a) / 65536;
        duty_b = 16384 + ((-vector_time.T1 - vector_time.T2) / 2);
        duty_b = (((int32_t)pHandle->PWMperiod) * duty_b) / 65536;
        duty_c = 16384 + ((vector_time.T1 + vector_time.T2) / 2);
        duty_c = (((int32_t)pHandle->PWMperiod) * duty_c) / 65536;
#endif
        pHandle->lowDuty = (uint16_t)duty_c;
        pHandle->midDuty = (uint16_t)duty_a;
        pHandle->highDuty = (uint16_t)duty_b;

      }
      else /* wZ >= 0 */
        if (wX <= 0)
        {
          pHandle->Sector = SECTOR_4;
          vector_time.T1 = wZ;
          vector_time.T2 = -wX;
          vector_time = PWMC_RecalcT1T2_OVM( vector_time, ovm_mode_flag, (int16_t)gama );
          duty_a = 16384 + ((- vector_time.T1 - vector_time.T2) / 2);
#ifndef FULL_MISRA_C_COMPLIANCY_PW_CURR_FDB_OVM
          //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
          duty_a = (((int32_t)pHandle->PWMperiod) * duty_a) >> 16;
          duty_b = 16384 + ((vector_time.T1 - vector_time.T2) / 2);
          //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
          duty_b = (((int32_t)pHandle->PWMperiod) * duty_b) >> 16;
          duty_c = 16384 + ((vector_time.T1 + vector_time.T2) / 2);
          //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
          duty_c = (((int32_t)pHandle->PWMperiod) * duty_c) >> 16;
#else
          duty_a = (((int32_t)pHandle->PWMperiod) * duty_a) / 65536;
          duty_b = 16384 + ((vector_time.T1 - vector_time.T2) / 2);
          duty_b = (((int32_t)pHandle->PWMperiod) * duty_b) / 65536;
          duty_c = 16384 + ((vector_time.T1 + vector_time.T2) / 2);
          duty_c = (((int32_t)pHandle->PWMperiod) * duty_c) / 65536;
#endif
          pHandle->lowDuty = (uint16_t)duty_c;
          pHandle->midDuty = (uint16_t)duty_b;
          pHandle->highDuty = (uint16_t)duty_a;
        }
        else /* wX > 0 */
        {
          pHandle->Sector = SECTOR_3;
          vector_time.T1 = wX;
          vector_time.T2 = -wY;
          vector_time = PWMC_RecalcT1T2_OVM( vector_time, ovm_mode_flag, (int16_t)gama );
          duty_a = 16384 + ((-vector_time.T1 - vector_time.T2) / 2);
#ifndef FULL_MISRA_C_COMPLIANCY_PW_CURR_FDB_OVM
          //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
          duty_a = (((int32_t)pHandle->PWMperiod) * duty_a) >> 16;
          duty_b = 16384 + ((vector_time.T1 + vector_time.T2) / 2);
          //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
          duty_b = (((int32_t)pHandle->PWMperiod) * duty_b) >> 16;
          duty_c = 16384 + ((- vector_time.T1 + vector_time.T2) / 2);
          //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
          duty_c = (((int32_t)pHandle->PWMperiod) * duty_c) >> 16;
#else
          duty_a = (((int32_t)pHandle->PWMperiod) * duty_a) / 65536;
          duty_b = 16384 + ((vector_time.T1 + vector_time.T2) / 2);
          duty_b = (((int32_t)pHandle->PWMperiod) * duty_b) / 65536;
          duty_c = 16384 + ((- vector_time.T1 + vector_time.T2) / 2);
          duty_c = (((int32_t)pHandle->PWMperiod) * duty_c) / 65536;
#endif
          pHandle->lowDuty = (uint16_t)duty_b;
          pHandle->midDuty = (uint16_t)duty_c;
          pHandle->highDuty = (uint16_t)duty_a;

        }
    }
    else /* wY > 0 */
    {
      if (wZ >= 0)
      {
        pHandle->Sector = SECTOR_2;
        vector_time.T1 = wY;
        vector_time.T2 = wZ;
        vector_time = PWMC_RecalcT1T2_OVM( vector_time, ovm_mode_flag, (int16_t)gama);
        duty_a = 16384 + ((vector_time.T1 - vector_time.T2) / 2);
#ifndef FULL_MISRA_C_COMPLIANCY_PW_CURR_FDB_OVM
        //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
        duty_a = (((int32_t)pHandle->PWMperiod) * duty_a) >> 16;
        duty_b = 16384 + ((vector_time.T1 + vector_time.T2) / 2);
        //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
        duty_b = (((int32_t)pHandle->PWMperiod) * duty_b) >> 16;
        duty_c = 16384 + ((-vector_time.T1 - vector_time.T2) / 2);
        //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
        duty_c = (((int32_t)pHandle->PWMperiod) * duty_c) >> 16;
#else
        duty_a = (((int32_t)pHandle->PWMperiod) * duty_a) / 65536;
        duty_b = 16384 + ((vector_time.T1 + vector_time.T2) / 2);
        duty_b = (((int32_t)pHandle->PWMperiod) * duty_b) / 65536;
        duty_c = 16384 + ((-vector_time.T1 - vector_time.T2) / 2);
        duty_c = (((int32_t)pHandle->PWMperiod) * duty_c) / 65536;
#endif
        pHandle->lowDuty = (uint16_t)duty_b;
        pHandle->midDuty = (uint16_t)duty_a;
        pHandle->highDuty = (uint16_t)duty_c;
      }
      else /* wZ < 0 */
        if (wX <= 0)
        {
          pHandle->Sector = SECTOR_6;
          vector_time.T1 = -wX;
          vector_time.T2 = wY;
          vector_time = PWMC_RecalcT1T2_OVM( vector_time, ovm_mode_flag, (int16_t)gama);
          duty_a = 16384 + ((vector_time.T1 + vector_time.T2) / 2);
#ifndef FULL_MISRA_C_COMPLIANCY_PW_CURR_FDB_OVM
          //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
          duty_a = (((int32_t)pHandle->PWMperiod) * duty_a) >> 16;
          duty_b = 16384 + ((-vector_time.T1 - vector_time.T2) / 2);
          //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
          duty_b = (((int32_t)pHandle->PWMperiod) * duty_b) >> 16;
          duty_c = 16384 + ((vector_time.T1 - vector_time.T2) / 2);
          //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
          duty_c = (((int32_t)pHandle->PWMperiod) * duty_c) >> 16;
#else
          duty_a = (((int32_t)pHandle->PWMperiod) * duty_a) / 65536;
          duty_b = 16384 + ((-vector_time.T1 - vector_time.T2) / 2);
          duty_b = (((int32_t)pHandle->PWMperiod) * duty_b) / 65536;
          duty_c = 16384 + ((vector_time.T1 - vector_time.T2) / 2);
          duty_c = (((int32_t)pHandle->PWMperiod) * duty_c) / 65536;
#endif
          pHandle->lowDuty = (uint16_t)duty_a;
          pHandle->midDuty = (uint16_t)duty_c;
          pHandle->highDuty = (uint16_t)duty_b;
        }
        else /* wX > 0 */
        {
          pHandle->Sector = SECTOR_1;
          vector_time.T1 = -wZ;
          vector_time.T2 = wX;
          vector_time = PWMC_RecalcT1T2_OVM( vector_time, ovm_mode_flag, (int16_t)gama);
          duty_a = 16384 + ((vector_time.T1 + vector_time.T2) / 2);
#ifndef FULL_MISRA_C_COMPLIANCY_PW_CURR_FDB_OVM
          //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
          duty_a = (((int32_t)pHandle->PWMperiod) * duty_a) >> 16;
          duty_b = 16384 + ((-vector_time.T1 + vector_time.T2) / 2);
          //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
          duty_b = (((int32_t)pHandle->PWMperiod) * duty_b) >> 16;
          duty_c = 16384 + ((-vector_time.T1 - vector_time.T2) / 2);
          //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
          duty_c = (((int32_t)pHandle->PWMperiod) * duty_c) >> 16;
#else
          duty_a = (((int32_t)pHandle->PWMperiod) * duty_a) / 65536;
          duty_b = 16384 + ((-vector_time.T1 + vector_time.T2) / 2);
          duty_b = (((int32_t)pHandle->PWMperiod) * duty_b) / 65536;
          duty_c = 16384 + ((-vector_time.T1 - vector_time.T2) / 2);
          duty_c = (((int32_t)pHandle->PWMperiod) * duty_c) / 65536;
#endif
          pHandle->lowDuty = (uint16_t)duty_a;
          pHandle->midDuty = (uint16_t)duty_b;
          pHandle->highDuty = (uint16_t)duty_c;
        }
    }

    pHandle->CntPhA = (uint16_t)duty_a ;
    pHandle->CntPhB = (uint16_t)duty_b ;
    pHandle->CntPhC = (uint16_t)duty_c;
    retvalue = pHandle->pFctSetADCSampPointSectX(pHandle);
#ifdef NULL_PTR_PWM_CUR_FDB_OVM
  }
#endif
  return (retvalue);
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
