/**
 ******************************************************************************
 * @file    sdmmc.c
 * @brief   This file provides code for the configuration
 *          of the SDMMC instances.
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
#include "sdmmc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SD_HandleTypeDef hsd1;

/* SDMMC1 init function */

void MX_SDMMC1_SD_Init(void)
{
    /* USER CODE BEGIN SDMMC1_Init 0 */

    /* USER CODE END SDMMC1_Init 0 */

    /* USER CODE BEGIN SDMMC1_Init 1 */

    /* USER CODE END SDMMC1_Init 1 */
    hsd1.Instance                 = SDMMC1;
    hsd1.Init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
    hsd1.Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    hsd1.Init.BusWide             = SDMMC_BUS_WIDE_4B;
    hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd1.Init.ClockDiv            = 0;
    hsd1.Init.TranceiverPresent   = SDMMC_TRANSCEIVER_NOT_PRESENT;
    /* USER CODE BEGIN SDMMC1_Init 2 */

    /* USER CODE END SDMMC1_Init 2 */
}

void HAL_SD_MspInit(SD_HandleTypeDef* sdHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct             = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    if (sdHandle->Instance == SDMMC1)
    {
        /* USER CODE BEGIN SDMMC1_MspInit 0 */

        /* USER CODE END SDMMC1_MspInit 0 */
        /** Initializes the peripherals clock
         */
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDMMC;
        PeriphClkInitStruct.SdmmcClockSelection  = RCC_SDMMCCLKSOURCE_PLL;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
        {
            Error_Handler();
        }

        /* SDMMC1 clock enable */
        __HAL_RCC_SDMMC1_CLK_ENABLE();

        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();
        /**SDMMC1 GPIO Configuration
        PC8     ------> SDMMC1_D0
        PC9     ------> SDMMC1_D1
        PC10     ------> SDMMC1_D2
        PC11     ------> SDMMC1_D3
        PC12     ------> SDMMC1_CK
        PD2     ------> SDMMC1_CMD
        */
        GPIO_InitStruct.Pin =
            SD_DAT0_Pin | SD_DAT1_Pin | SD_DAT2_Pin | SD_DAT3_Pin | SD_CLK_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF12_SDIO1;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin       = SD_CMD_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF12_SDIO1;
        HAL_GPIO_Init(SD_CMD_GPIO_Port, &GPIO_InitStruct);

        /* USER CODE BEGIN SDMMC1_MspInit 1 */

        /* USER CODE END SDMMC1_MspInit 1 */
    }
}

void HAL_SD_MspDeInit(SD_HandleTypeDef* sdHandle)
{
    if (sdHandle->Instance == SDMMC1)
    {
        /* USER CODE BEGIN SDMMC1_MspDeInit 0 */

        /* USER CODE END SDMMC1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_SDMMC1_CLK_DISABLE();

        /**SDMMC1 GPIO Configuration
        PC8     ------> SDMMC1_D0
        PC9     ------> SDMMC1_D1
        PC10     ------> SDMMC1_D2
        PC11     ------> SDMMC1_D3
        PC12     ------> SDMMC1_CK
        PD2     ------> SDMMC1_CMD
        */
        HAL_GPIO_DeInit(
            GPIOC, SD_DAT0_Pin | SD_DAT1_Pin | SD_DAT2_Pin | SD_DAT3_Pin | SD_CLK_Pin);

        HAL_GPIO_DeInit(SD_CMD_GPIO_Port, SD_CMD_Pin);

        /* USER CODE BEGIN SDMMC1_MspDeInit 1 */

        /* USER CODE END SDMMC1_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
