/**
 ******************************************************************************
 * @file    i2c.c
 * @brief   This file provides code for the configuration
 *          of the I2C instances.
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
#include "i2c.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SMBUS_HandleTypeDef hsmbus2;

/* I2C2 init function */

void MX_I2C2_SMBUS_Init(void)
{
    /* USER CODE BEGIN I2C2_Init 0 */

    /* USER CODE END I2C2_Init 0 */

    /* USER CODE BEGIN I2C2_Init 1 */

    /* USER CODE END I2C2_Init 1 */
    hsmbus2.Instance                  = I2C2;
    hsmbus2.Init.Timing               = 0x10707DBC;
    hsmbus2.Init.AnalogFilter         = SMBUS_ANALOGFILTER_ENABLE;
    hsmbus2.Init.OwnAddress1          = 2;
    hsmbus2.Init.AddressingMode       = SMBUS_ADDRESSINGMODE_7BIT;
    hsmbus2.Init.DualAddressMode      = SMBUS_DUALADDRESS_DISABLE;
    hsmbus2.Init.OwnAddress2          = 0;
    hsmbus2.Init.OwnAddress2Masks     = SMBUS_OA2_NOMASK;
    hsmbus2.Init.GeneralCallMode      = SMBUS_GENERALCALL_DISABLE;
    hsmbus2.Init.NoStretchMode        = SMBUS_NOSTRETCH_DISABLE;
    hsmbus2.Init.PacketErrorCheckMode = SMBUS_PEC_DISABLE;
    hsmbus2.Init.PeripheralMode       = SMBUS_PERIPHERAL_MODE_SMBUS_SLAVE;
    hsmbus2.Init.SMBusTimeout         = 0x0000830D;
    if (HAL_SMBUS_Init(&hsmbus2) != HAL_OK)
    {
        Error_Handler();
    }
    /** configuration Alert Mode
     */
    if (HAL_SMBUS_EnableAlert_IT(&hsmbus2) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure Digital filter
     */
    if (HAL_SMBUS_ConfigDigitalFilter(&hsmbus2, 0) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C2_Init 2 */

    /* USER CODE END I2C2_Init 2 */
}

void HAL_SMBUS_MspInit(SMBUS_HandleTypeDef* smbusHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct             = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    if (smbusHandle->Instance == I2C2)
    {
        /* USER CODE BEGIN I2C2_MspInit 0 */

        /* USER CODE END I2C2_MspInit 0 */
        /** Initializes the peripherals clock
         */
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
        PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
        {
            Error_Handler();
        }

        __HAL_RCC_GPIOF_CLK_ENABLE();
        /**I2C2 GPIO Configuration
        PF0     ------> I2C2_SDA
        PF1     ------> I2C2_SCL
        PF2     ------> I2C2_SMBA
        */
        GPIO_InitStruct.Pin = power_monitor_I2C2_SDA_Pin | power_monitor_I2C2_SCL_Pin |
                              power_monitor_I2C2_SMBA_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
        HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

        /* I2C2 clock enable */
        __HAL_RCC_I2C2_CLK_ENABLE();
        /* USER CODE BEGIN I2C2_MspInit 1 */

        /* USER CODE END I2C2_MspInit 1 */
    }
}

void HAL_SMBUS_MspDeInit(SMBUS_HandleTypeDef* smbusHandle)
{
    if (smbusHandle->Instance == I2C2)
    {
        /* USER CODE BEGIN I2C2_MspDeInit 0 */

        /* USER CODE END I2C2_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_I2C2_CLK_DISABLE();

        /**I2C2 GPIO Configuration
        PF0     ------> I2C2_SDA
        PF1     ------> I2C2_SCL
        PF2     ------> I2C2_SMBA
        */
        HAL_GPIO_DeInit(power_monitor_I2C2_SDA_GPIO_Port, power_monitor_I2C2_SDA_Pin);

        HAL_GPIO_DeInit(power_monitor_I2C2_SCL_GPIO_Port, power_monitor_I2C2_SCL_Pin);

        HAL_GPIO_DeInit(power_monitor_I2C2_SMBA_GPIO_Port, power_monitor_I2C2_SMBA_Pin);

        /* USER CODE BEGIN I2C2_MspDeInit 1 */

        /* USER CODE END I2C2_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
