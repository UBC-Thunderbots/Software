
/**
 ******************************************************************************
 * @file    stm32f0xx_mc_it.c
 * @author  Motor Control SDK Team, ST Microelectronics
 * @brief   Main Interrupt Service Routines.
 *          This file provides exceptions handler and peripherals interrupt
 *          service routine related to Motor Control for the STM32F0 Family.
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
 * @ingroup STM32F0xx_IRQ_Handlers
 */

/* Includes ------------------------------------------------------------------*/
#include "firmware/motor/mc_config.h"
#include "firmware/motor/mc_tasks.h"
#include "firmware/motor/mcsdk/mc_type.h"
#include "firmware/motor/motorcontrol.h"
#include "firmware/motor/parameters_conversion.h"
#include "firmware/motor/stm32f0xx/stm32f0xx.h"
#include "firmware/motor/stm32f0xx/stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup STM32F0xx_IRQ_Handlers STM32F0xx IRQ Handlers
 * @{
 */

/* USER CODE BEGIN PRIVATE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USER CODE END PRIVATE */

/* Public prototypes of IRQ handlers called from assembly code ---------------*/
void CURRENT_REGULATION_IRQHandler(void);
void DMAx_R1_M1_IRQHandler(void);
void TIMx_UP_BRK_M1_IRQHandler(void);
void SPD_TIM_M1_IRQHandler(void);

/**
 * @brief  This function handles current regulation interrupt request.
 * @param  None
 */
void CURRENT_REGULATION_IRQHandler(void)
{
    /* USER CODE BEGIN CURRENT_REGULATION_IRQn 0 */

    /* USER CODE END CURRENT_REGULATION_IRQn 0 */

    /* Clear Flags */
    DMA1->IFCR = (LL_DMA_ISR_GIF1 | LL_DMA_ISR_TCIF1 | LL_DMA_ISR_HTIF1);

    /* USER CODE BEGIN CURRENT_REGULATION_IRQn 1 */

    /* USER CODE END CURRENT_REGULATION_IRQn 1 */
    TSK_HighFrequencyTask();

    /* USER CODE BEGIN CURRENT_REGULATION_IRQn 2 */

    /* USER CODE END CURRENT_REGULATION_IRQn 2 */
}

/**
 * @brief  This function handles first motor TIMx Update, Break-in interrupt request.
 * @param  None
 */
void TIMx_UP_BRK_M1_IRQHandler(void)
{
    /* USER CODE BEGIN TIMx_UP_BRK_M1_IRQn 0 */

    /* USER CODE END TIMx_UP_BRK_M1_IRQn 0 */

    if (LL_TIM_IsActiveFlag_UPDATE(TIM1) && LL_TIM_IsEnabledIT_UPDATE(TIM1))
    {
        R1_TIM1_UP_IRQHandler(&PWM_Handle_M1);
        LL_TIM_ClearFlag_UPDATE(TIM1);

        /* USER CODE BEGIN PWM_Update */

        /* USER CODE END PWM_Update */
    }
    else
    {
        /* Nothing to do */
    }

    if (LL_TIM_IsActiveFlag_BRK(TIM1) && LL_TIM_IsEnabledIT_BRK(TIM1))
    {
        LL_TIM_ClearFlag_BRK(TIM1);
        PWMC_OCP_Handler(&PWM_Handle_M1._Super);

        /* USER CODE BEGIN Break */

        /* USER CODE END Break */
    }
    else
    {
        /* No other interrupts are routed to this handler */
    }

    /* USER CODE BEGIN TIMx_UP_BRK_M1_IRQn 1 */

    /* USER CODE END TIMx_UP_BRK_M1_IRQn 1 */
}

/**
 * @brief  This function handles first motor DMAx TC interrupt request.
 *         Required only for R1 with rep rate > 1
 * @param  None
 */
void DMAx_R1_M1_IRQHandler(void)
{
    /* USER CODE BEGIN DMAx_R1_M1_IRQn 0 */

    /* USER CODE END DMAx_R1_M1_IRQn 0 */

    if (LL_DMA_IsActiveFlag_TC5(DMA1))
    {
        LL_DMA_ClearFlag_TC5(DMA1);
        R1_DMAx_TC_IRQHandler(&PWM_Handle_M1);
        /* USER CODE BEGIN DMAx_R1_M1_TC5 */

        /* USER CODE END DMAx_R1_M1_TC5 */
    }
    else
    {
        /* Nothing to do */
    }

    /* USER CODE BEGIN DMAx_R1_M1_IRQn 1 */

    /* USER CODE END DMAx_R1_M1_IRQn 1 */
}

/**
 * @brief  This function handles TIMx global interrupt request for M1 Speed Sensor.
 * @param  None
 */
void SPD_TIM_M1_IRQHandler(void)
{
    /* USER CODE BEGIN SPD_TIM_M1_IRQn 0 */

    /* USER CODE END SPD_TIM_M1_IRQn 0 */

    /* HALL Timer Update IT always enabled, no need to check enable UPDATE state */
    if (LL_TIM_IsActiveFlag_UPDATE(HALL_M1.TIMx) != 0)
    {
        LL_TIM_ClearFlag_UPDATE(HALL_M1.TIMx);
        HALL_TIMx_UP_IRQHandler(&HALL_M1);

        /* USER CODE BEGIN HALL_Update */

        /* USER CODE END HALL_Update   */
    }
    else
    {
        /* Nothing to do */
    }

    /* HALL Timer CC1 IT always enabled, no need to check enable CC1 state */
    if (LL_TIM_IsActiveFlag_CC1(HALL_M1.TIMx))
    {
        LL_TIM_ClearFlag_CC1(HALL_M1.TIMx);
        HALL_TIMx_CC_IRQHandler(&HALL_M1);

        /* USER CODE BEGIN HALL_CC1 */

        /* USER CODE END HALL_CC1 */
    }
    else
    {
        /* Nothing to do */
    }

    /* USER CODE BEGIN SPD_TIM_M1_IRQn 1 */

    /* USER CODE END SPD_TIM_M1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
 * @}
 */

/**
 * @}
 */

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
