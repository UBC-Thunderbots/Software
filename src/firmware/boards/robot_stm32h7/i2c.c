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

SMBUS_HandleTypeDef hsmbus1;
I2C_HandleTypeDef hi2c2;

/* I2C1 init function */

void MX_I2C1_SMBUS_Init(void)
{
    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hsmbus1.Instance                  = I2C1;
    hsmbus1.Init.Timing               = 0x307075B1;
    hsmbus1.Init.AnalogFilter         = SMBUS_ANALOGFILTER_ENABLE;
    hsmbus1.Init.OwnAddress1          = 2;
    hsmbus1.Init.AddressingMode       = SMBUS_ADDRESSINGMODE_7BIT;
    hsmbus1.Init.DualAddressMode      = SMBUS_DUALADDRESS_DISABLE;
    hsmbus1.Init.OwnAddress2          = 0;
    hsmbus1.Init.OwnAddress2Masks     = SMBUS_OA2_NOMASK;
    hsmbus1.Init.GeneralCallMode      = SMBUS_GENERALCALL_DISABLE;
    hsmbus1.Init.NoStretchMode        = SMBUS_NOSTRETCH_DISABLE;
    hsmbus1.Init.PacketErrorCheckMode = SMBUS_PEC_DISABLE;
    hsmbus1.Init.PeripheralMode       = SMBUS_PERIPHERAL_MODE_SMBUS_SLAVE;
    hsmbus1.Init.SMBusTimeout         = 0x000085B8;
    if (HAL_SMBUS_Init(&hsmbus1) != HAL_OK)
    {
        Error_Handler();
    }
    /** configuration Alert Mode
     */
    if (HAL_SMBUS_EnableAlert_IT(&hsmbus1) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure Digital filter
     */
    if (HAL_SMBUS_ConfigDigitalFilter(&hsmbus1, 0) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */
}
/* I2C2 init function */
void MX_I2C2_Init(void)
{
    /* USER CODE BEGIN I2C2_Init 0 */

    /* USER CODE END I2C2_Init 0 */

    /* USER CODE BEGIN I2C2_Init 1 */

    /* USER CODE END I2C2_Init 1 */
    hi2c2.Instance              = I2C2;
    hi2c2.Init.Timing           = 0x307075B1;
    hi2c2.Init.OwnAddress1      = 0;
    hi2c2.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2      = 0;
    hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c2.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure Analogue filter
     */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure Digital filter
     */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
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
    if (smbusHandle->Instance == I2C1)
    {
        /* USER CODE BEGIN I2C1_MspInit 0 */

        /* USER CODE END I2C1_MspInit 0 */
        /** Initializes the peripherals clock
         */
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
        PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
        {
            Error_Handler();
        }

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**I2C1 GPIO Configuration
        PB5     ------> I2C1_SMBA
        PB6     ------> I2C1_SCL
        PB7     ------> I2C1_SDA
        */
        GPIO_InitStruct.Pin =
            PMS_ALERT_PWR_BRD_Pin | PMS_SCL_PWR_BRD_Pin | PMS_SDA_PWR_BRD_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* I2C1 clock enable */
        __HAL_RCC_I2C1_CLK_ENABLE();
        /* USER CODE BEGIN I2C1_MspInit 1 */

        /* USER CODE END I2C1_MspInit 1 */
    }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct             = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    if (i2cHandle->Instance == I2C2)
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
        */
        GPIO_InitStruct.Pin       = IMU_SDA_Pin | IMU_SCL_Pin;
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
    if (smbusHandle->Instance == I2C1)
    {
        /* USER CODE BEGIN I2C1_MspDeInit 0 */

        /* USER CODE END I2C1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_I2C1_CLK_DISABLE();

        /**I2C1 GPIO Configuration
        PB5     ------> I2C1_SMBA
        PB6     ------> I2C1_SCL
        PB7     ------> I2C1_SDA
        */
        HAL_GPIO_DeInit(PMS_ALERT_PWR_BRD_GPIO_Port, PMS_ALERT_PWR_BRD_Pin);

        HAL_GPIO_DeInit(PMS_SCL_PWR_BRD_GPIO_Port, PMS_SCL_PWR_BRD_Pin);

        HAL_GPIO_DeInit(PMS_SDA_PWR_BRD_GPIO_Port, PMS_SDA_PWR_BRD_Pin);

        /* USER CODE BEGIN I2C1_MspDeInit 1 */

        /* USER CODE END I2C1_MspDeInit 1 */
    }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{
    if (i2cHandle->Instance == I2C2)
    {
        /* USER CODE BEGIN I2C2_MspDeInit 0 */

        /* USER CODE END I2C2_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_I2C2_CLK_DISABLE();

        /**I2C2 GPIO Configuration
        PF0     ------> I2C2_SDA
        PF1     ------> I2C2_SCL
        */
        HAL_GPIO_DeInit(IMU_SDA_GPIO_Port, IMU_SDA_Pin);

        HAL_GPIO_DeInit(IMU_SCL_GPIO_Port, IMU_SCL_Pin);

        /* USER CODE BEGIN I2C2_MspDeInit 1 */

        /* USER CODE END I2C2_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
