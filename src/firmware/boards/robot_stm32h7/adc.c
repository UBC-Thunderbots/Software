/**
 ******************************************************************************
 * @file    adc.c
 * @brief   This file provides code for the configuration
 *          of the ADC instances.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "adc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

/* ADC1 init function */
void MX_ADC1_Init(void)
{
    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */
    /** Common config
     */
    hadc1.Instance                      = ADC1;
    hadc1.Init.ClockPrescaler           = ADC_CLOCK_ASYNC_DIV2;
    hadc1.Init.Resolution               = ADC_RESOLUTION_16B;
    hadc1.Init.ScanConvMode             = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait         = DISABLE;
    hadc1.Init.ContinuousConvMode       = DISABLE;
    hadc1.Init.NbrOfConversion          = 1;
    hadc1.Init.DiscontinuousConvMode    = DISABLE;
    hadc1.Init.ExternalTrigConv         = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    hadc1.Init.Overrun                  = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.LeftBitShift             = ADC_LEFTBITSHIFT_NONE;
    hadc1.Init.OversamplingMode         = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure the ADC multi-mode
     */
    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure Regular Channel
     */
    sConfig.Channel                = ADC_CHANNEL_2;
    sConfig.Rank                   = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime           = ADC_SAMPLETIME_1CYCLE_5;
    sConfig.SingleDiff             = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber           = ADC_OFFSET_NONE;
    sConfig.Offset                 = 0;
    sConfig.OffsetSignedSaturation = DISABLE;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */
}
/* ADC2 init function */
void MX_ADC2_Init(void)
{
    /* USER CODE BEGIN ADC2_Init 0 */

    /* USER CODE END ADC2_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC2_Init 1 */

    /* USER CODE END ADC2_Init 1 */
    /** Common config
     */
    hadc2.Instance                      = ADC2;
    hadc2.Init.ClockPrescaler           = ADC_CLOCK_ASYNC_DIV2;
    hadc2.Init.Resolution               = ADC_RESOLUTION_16B;
    hadc2.Init.ScanConvMode             = ADC_SCAN_DISABLE;
    hadc2.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
    hadc2.Init.LowPowerAutoWait         = DISABLE;
    hadc2.Init.ContinuousConvMode       = DISABLE;
    hadc2.Init.NbrOfConversion          = 1;
    hadc2.Init.DiscontinuousConvMode    = DISABLE;
    hadc2.Init.ExternalTrigConv         = ADC_SOFTWARE_START;
    hadc2.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    hadc2.Init.Overrun                  = ADC_OVR_DATA_PRESERVED;
    hadc2.Init.LeftBitShift             = ADC_LEFTBITSHIFT_NONE;
    hadc2.Init.OversamplingMode         = DISABLE;
    if (HAL_ADC_Init(&hadc2) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure Regular Channel
     */
    sConfig.Channel                = ADC_CHANNEL_10;
    sConfig.Rank                   = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime           = ADC_SAMPLETIME_1CYCLE_5;
    sConfig.SingleDiff             = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber           = ADC_OFFSET_NONE;
    sConfig.Offset                 = 0;
    sConfig.OffsetSignedSaturation = DISABLE;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC2_Init 2 */

    /* USER CODE END ADC2_Init 2 */
}
/* ADC3 init function */
void MX_ADC3_Init(void)
{
    /* USER CODE BEGIN ADC3_Init 0 */

    /* USER CODE END ADC3_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC3_Init 1 */

    /* USER CODE END ADC3_Init 1 */
    /** Common config
     */
    hadc3.Instance                      = ADC3;
    hadc3.Init.ClockPrescaler           = ADC_CLOCK_ASYNC_DIV2;
    hadc3.Init.Resolution               = ADC_RESOLUTION_16B;
    hadc3.Init.ScanConvMode             = ADC_SCAN_DISABLE;
    hadc3.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
    hadc3.Init.LowPowerAutoWait         = DISABLE;
    hadc3.Init.ContinuousConvMode       = DISABLE;
    hadc3.Init.NbrOfConversion          = 1;
    hadc3.Init.DiscontinuousConvMode    = DISABLE;
    hadc3.Init.ExternalTrigConv         = ADC_SOFTWARE_START;
    hadc3.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    hadc3.Init.Overrun                  = ADC_OVR_DATA_PRESERVED;
    hadc3.Init.LeftBitShift             = ADC_LEFTBITSHIFT_NONE;
    hadc3.Init.OversamplingMode         = DISABLE;
    if (HAL_ADC_Init(&hadc3) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure Regular Channel
     */
    sConfig.Channel                = ADC_CHANNEL_1;
    sConfig.Rank                   = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime           = ADC_SAMPLETIME_1CYCLE_5;
    sConfig.SingleDiff             = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber           = ADC_OFFSET_NONE;
    sConfig.Offset                 = 0;
    sConfig.OffsetSignedSaturation = DISABLE;
    if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC3_Init 2 */

    /* USER CODE END ADC3_Init 2 */
}

static uint32_t HAL_RCC_ADC12_CLK_ENABLED = 0;

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (adcHandle->Instance == ADC1)
    {
        /* USER CODE BEGIN ADC1_MspInit 0 */

        /* USER CODE END ADC1_MspInit 0 */
        /* ADC1 clock enable */
        HAL_RCC_ADC12_CLK_ENABLED++;
        if (HAL_RCC_ADC12_CLK_ENABLED == 1)
        {
            __HAL_RCC_ADC12_CLK_ENABLE();
        }

        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOF_CLK_ENABLE();
        /**ADC1 GPIO Configuration
        PC0     ------> ADC1_INP10
        PA3     ------> ADC1_INP15
        PB0     ------> ADC1_INP9
        PF11     ------> ADC1_INP2
        PF12     ------> ADC1_INP6
        */
        GPIO_InitStruct.Pin  = WHEEL_FRONT_RIGHT_CSOUT_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(WHEEL_FRONT_RIGHT_CSOUT_GPIO_Port, &GPIO_InitStruct);

        GPIO_InitStruct.Pin  = WHEEL_BACK_RIGHT_CSOUT_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(WHEEL_BACK_RIGHT_CSOUT_GPIO_Port, &GPIO_InitStruct);

        GPIO_InitStruct.Pin  = DRIBBLER_CSOUT_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(DRIBBLER_CSOUT_GPIO_Port, &GPIO_InitStruct);

        GPIO_InitStruct.Pin  = WHEEL_FRONT_LEFT_CSOUT_Pin | WHEEL_BACK_LEFT_CSOUT_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

        /* USER CODE BEGIN ADC1_MspInit 1 */

        /* USER CODE END ADC1_MspInit 1 */
    }
    else if (adcHandle->Instance == ADC2)
    {
        /* USER CODE BEGIN ADC2_MspInit 0 */

        /* USER CODE END ADC2_MspInit 0 */
        /* ADC2 clock enable */
        HAL_RCC_ADC12_CLK_ENABLED++;
        if (HAL_RCC_ADC12_CLK_ENABLED == 1)
        {
            __HAL_RCC_ADC12_CLK_ENABLE();
        }

        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**ADC2 GPIO Configuration
        PC0     ------> ADC2_INP10
        PA3     ------> ADC2_INP15
        PB0     ------> ADC2_INP9
        */
        GPIO_InitStruct.Pin  = WHEEL_FRONT_RIGHT_CSOUT_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(WHEEL_FRONT_RIGHT_CSOUT_GPIO_Port, &GPIO_InitStruct);

        GPIO_InitStruct.Pin  = WHEEL_BACK_RIGHT_CSOUT_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(WHEEL_BACK_RIGHT_CSOUT_GPIO_Port, &GPIO_InitStruct);

        GPIO_InitStruct.Pin  = DRIBBLER_CSOUT_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(DRIBBLER_CSOUT_GPIO_Port, &GPIO_InitStruct);

        /* USER CODE BEGIN ADC2_MspInit 1 */

        /* USER CODE END ADC2_MspInit 1 */
    }
    else if (adcHandle->Instance == ADC3)
    {
        /* USER CODE BEGIN ADC3_MspInit 0 */

        /* USER CODE END ADC3_MspInit 0 */
        /* ADC3 clock enable */
        __HAL_RCC_ADC3_CLK_ENABLE();

        __HAL_RCC_GPIOF_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        /**ADC3 GPIO Configuration
        PF3     ------> ADC3_INP5
        PF4     ------> ADC3_INP9
        PF5     ------> ADC3_INP4
        PF6     ------> ADC3_INN4
        PF6     ------> ADC3_INP8
        PF7     ------> ADC3_INP3
        PF8     ------> ADC3_INN3
        PF8     ------> ADC3_INP7
        PF9     ------> ADC3_INP2
        PC0     ------> ADC3_INP10
        PC2_C     ------> ADC3_INN1
        PC2_C     ------> ADC3_INP0
        PC3_C     ------> ADC3_INP1
        */
        GPIO_InitStruct.Pin = ENCODER_FRONT_LEFT_SIN_Pin | ENCODER_FRONT_LEFT_COS_Pin |
                              ENCODER_BACK_LEFT_COS_Pin | ENCODER_BACK_LEFT_SIN_Pin |
                              ENCODER_BACK_RIGHT_COS_Pin | ENCODER_BACK_RIGHT_SIN_Pin |
                              HV_SENSE_PWR_BRD_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

        GPIO_InitStruct.Pin  = WHEEL_FRONT_RIGHT_CSOUT_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(WHEEL_FRONT_RIGHT_CSOUT_GPIO_Port, &GPIO_InitStruct);

        HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC2, SYSCFG_SWITCH_PC2_OPEN);

        HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3, SYSCFG_SWITCH_PC3_OPEN);

        /* USER CODE BEGIN ADC3_MspInit 1 */

        /* USER CODE END ADC3_MspInit 1 */
    }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{
    if (adcHandle->Instance == ADC1)
    {
        /* USER CODE BEGIN ADC1_MspDeInit 0 */

        /* USER CODE END ADC1_MspDeInit 0 */
        /* Peripheral clock disable */
        HAL_RCC_ADC12_CLK_ENABLED--;
        if (HAL_RCC_ADC12_CLK_ENABLED == 0)
        {
            __HAL_RCC_ADC12_CLK_DISABLE();
        }

        /**ADC1 GPIO Configuration
        PC0     ------> ADC1_INP10
        PA3     ------> ADC1_INP15
        PB0     ------> ADC1_INP9
        PF11     ------> ADC1_INP2
        PF12     ------> ADC1_INP6
        */
        HAL_GPIO_DeInit(WHEEL_FRONT_RIGHT_CSOUT_GPIO_Port, WHEEL_FRONT_RIGHT_CSOUT_Pin);

        HAL_GPIO_DeInit(WHEEL_BACK_RIGHT_CSOUT_GPIO_Port, WHEEL_BACK_RIGHT_CSOUT_Pin);

        HAL_GPIO_DeInit(DRIBBLER_CSOUT_GPIO_Port, DRIBBLER_CSOUT_Pin);

        HAL_GPIO_DeInit(GPIOF, WHEEL_FRONT_LEFT_CSOUT_Pin | WHEEL_BACK_LEFT_CSOUT_Pin);

        /* USER CODE BEGIN ADC1_MspDeInit 1 */

        /* USER CODE END ADC1_MspDeInit 1 */
    }
    else if (adcHandle->Instance == ADC2)
    {
        /* USER CODE BEGIN ADC2_MspDeInit 0 */

        /* USER CODE END ADC2_MspDeInit 0 */
        /* Peripheral clock disable */
        HAL_RCC_ADC12_CLK_ENABLED--;
        if (HAL_RCC_ADC12_CLK_ENABLED == 0)
        {
            __HAL_RCC_ADC12_CLK_DISABLE();
        }

        /**ADC2 GPIO Configuration
        PC0     ------> ADC2_INP10
        PA3     ------> ADC2_INP15
        PB0     ------> ADC2_INP9
        */
        HAL_GPIO_DeInit(WHEEL_FRONT_RIGHT_CSOUT_GPIO_Port, WHEEL_FRONT_RIGHT_CSOUT_Pin);

        HAL_GPIO_DeInit(WHEEL_BACK_RIGHT_CSOUT_GPIO_Port, WHEEL_BACK_RIGHT_CSOUT_Pin);

        HAL_GPIO_DeInit(DRIBBLER_CSOUT_GPIO_Port, DRIBBLER_CSOUT_Pin);

        /* USER CODE BEGIN ADC2_MspDeInit 1 */

        /* USER CODE END ADC2_MspDeInit 1 */
    }
    else if (adcHandle->Instance == ADC3)
    {
        /* USER CODE BEGIN ADC3_MspDeInit 0 */

        /* USER CODE END ADC3_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_ADC3_CLK_DISABLE();

        /**ADC3 GPIO Configuration
        PF3     ------> ADC3_INP5
        PF4     ------> ADC3_INP9
        PF5     ------> ADC3_INP4
        PF6     ------> ADC3_INN4
        PF6     ------> ADC3_INP8
        PF7     ------> ADC3_INP3
        PF8     ------> ADC3_INN3
        PF8     ------> ADC3_INP7
        PF9     ------> ADC3_INP2
        PC0     ------> ADC3_INP10
        PC2_C     ------> ADC3_INN1
        PC2_C     ------> ADC3_INP0
        PC3_C     ------> ADC3_INP1
        */
        HAL_GPIO_DeInit(GPIOF, ENCODER_FRONT_LEFT_SIN_Pin | ENCODER_FRONT_LEFT_COS_Pin |
                                   ENCODER_BACK_LEFT_COS_Pin | ENCODER_BACK_LEFT_SIN_Pin |
                                   ENCODER_BACK_RIGHT_COS_Pin |
                                   ENCODER_BACK_RIGHT_SIN_Pin | HV_SENSE_PWR_BRD_Pin);

        HAL_GPIO_DeInit(WHEEL_FRONT_RIGHT_CSOUT_GPIO_Port, WHEEL_FRONT_RIGHT_CSOUT_Pin);

        /* USER CODE BEGIN ADC3_MspDeInit 1 */

        /* USER CODE END ADC3_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
