/**
  ******************************************************************************
  * @file    max_torque_per_ampere.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Maximum torque per ampere (MTPA) control for I-PMSM motors
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
  * @ingroup MTPA
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MAXTORQUEPERAMPERE_H
#define MAXTORQUEPERAMPERE_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup MTPA Maximum Torque Per Ampere Control
  * @{
  */

/* Exported types ------------------------------------------------------------*/
#define SEGMENT_NUM         7U               /* coeff no. -1 */
#define MTPA_ARRAY_SIZE     SEGMENT_NUM + 1U
/**
  * @brief  Handle structure of max_torque_per_ampere.c
  */
typedef struct
{
  int16_t  SegDiv;               /**< Segments divisor */
  int32_t  AngCoeff[MTPA_ARRAY_SIZE];          /**< Angular coefficients table */
  int32_t  Offset[MTPA_ARRAY_SIZE];            /**< Offsets table */
} MTPA_Handle_t;

/* Exported functions ------------------------------------------------------- */

/* Calculates the Id current ref based on input Iq current */
void MTPA_CalcCurrRefFromIq(const MTPA_Handle_t *pHandle, qd_t *Iqdref);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* MAXTORQUEPERAMPERE_H */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
