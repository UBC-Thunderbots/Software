/**
  ******************************************************************************
  * @file    circle_limitation.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides the functions that implement the circle
  *          limitation feature of the STM32 Motor Control SDK.
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
  * @ingroup CircleLimitation
  */

/* Includes ------------------------------------------------------------------*/
#include "circle_limitation.h"
#include "mc_math.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup CircleLimitation Circle Limitation
  * @brief Circle Limitation component of the Motor Control SDK
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

#if defined CIRCLE_LIMITATION_SQRT_M0
const uint16_t SqrtTable[1025] = SQRT_CIRCLE_LIMITATION;
#endif
/**
  * @brief  Returns the saturated @f$v_q, v_d@f$ component values
  * @param  pHandle Handler of the CircleLimitation component
  * @param  Vqd @f$v_q, v_d@f$ values
  * @retval Saturated @f$v_q, v_d@f$ values
  *
  * This function implements the CircleLimitation feature described CircleLimitation component.
  *
  *  @f$v_d = \min(v_d^*, v_d MAX) @f$
  *
  *  @f$v_q = \sqrt(MaxModule^2-v_d^2\ ) @f$

  *
  */
__weak qd_t Circle_Limitation(const CircleLimitation_Handle_t *pHandle, qd_t Vqd)
{
  qd_t Local_Vqd = Vqd;
#ifdef NULL_PTR_CHECK_CRC_LIM
  if (MC_NULL == pHandle)
  {
    Local_Vqd.q = 0;
    Local_Vqd.d = 0;
  }
  else
  {
#endif
    int32_t maxModule;
    int32_t square_q;
    int32_t square_temp;
    int32_t square_d;
    int32_t square_sum;
    int32_t square_limit;
    int32_t vd_square_limit;
    int32_t new_q;
    int32_t new_d;

    maxModule = (int32_t)pHandle->MaxModule;

    square_q = ((int32_t)(Vqd.q)) * Vqd.q;
    square_d = ((int32_t)(Vqd.d)) * Vqd.d;
    square_limit = maxModule * maxModule;
    vd_square_limit = ((int32_t)pHandle->MaxVd) * ((int32_t)pHandle->MaxVd);
    square_sum = square_q + square_d;

    if (square_sum > square_limit)
    {
      if (square_d <= vd_square_limit)
      {
#if defined CIRCLE_LIMITATION_SQRT_M0
        square_temp = (square_limit - square_d) / 1048576;
        new_q = SqrtTable[square_temp];
#else
        square_temp = square_limit - square_d;
        new_q = MCM_Sqrt(square_temp);
#endif
        if (Vqd.q < 0)
        {
          new_q = -new_q;
        }
        new_d = Vqd.d;
      }
      else
      {
        new_d = (int32_t)pHandle->MaxVd;
        if (Vqd.d < 0)
        {
          new_d = -new_d;
        }
#if defined CIRCLE_LIMITATION_SQRT_M0
        square_temp = (square_limit - vd_square_limit) / 1048576;
        new_q = SqrtTable[square_temp];
#else
        square_temp = square_limit - vd_square_limit;
        new_q = MCM_Sqrt(square_temp);
#endif
        if (Vqd.q < 0)
        {
          new_q = - new_q;
        }
      }
      Local_Vqd.q = (int16_t)new_q;
      Local_Vqd.d = (int16_t)new_d;
    }
#ifdef NULL_PTR_CHECK_CRC_LIM
  }
#endif
  return (Local_Vqd);
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/

