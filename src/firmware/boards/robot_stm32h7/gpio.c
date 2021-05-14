/**
 ******************************************************************************
 * @file    gpio.c
 * @brief   This file provides code for the configuration
 *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
     PC14-OSC32_IN (OSC32_IN)   ------> RCC_OSC32_IN
     PC15-OSC32_OUT (OSC32_OUT)   ------> RCC_OSC32_OUT
     PH0-OSC_IN (PH0)   ------> RCC_OSC_IN
     PH1-OSC_OUT (PH1)   ------> RCC_OSC_OUT
     PC8   ------> SDMMC1_D0
     PC9   ------> SDMMC1_D1
     PA13 (JTMS/SWDIO)   ------> DEBUG_JTMS-SWDIO
     PA14 (JTCK/SWCLK)   ------> DEBUG_JTCK-SWCLK
     PC10   ------> SDMMC1_D2
     PC11   ------> SDMMC1_D3
     PC12   ------> SDMMC1_CK
     PD2   ------> SDMMC1_CMD
     PB3 (JTDO/TRACESWO)   ------> DEBUG_JTDO-SWO
*/
void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOE,
                      CHARGE_PWR_BRD_Pin | WHEEL_FRONT_RIGHT_FF1_Pin | IMU_INT1_Pin |
                          USER_LED_2_Pin | USER_LED_3_Pin | WHEEL_BACK_RIGHT_DIR_Pin |
                          WHEEL_BACK_RIGHT_FF1_Pin,
                      GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(IMU_INT2_GPIO_Port, IMU_INT2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOF,
                      WHEEL_FRONT_RIGHT_RESET_Pin | WHEEL_FRONT_RIGHT_FF2_Pin |
                          RED_IN_RGB_Pin | BLUE_IN_RGB_Pin | GREEN_IN_RGB_Pin,
                      GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(WHEEL_BACK_RIGHT_RESET_GPIO_Port, WHEEL_BACK_RIGHT_RESET_Pin,
                      GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOD,
                      DRIBBLER_RESET_Pin | WHEEL_FRONT_LEFT_MODE_Pin |
                          WHEEL_FRONT_LEFT_DIR_Pin | DRIBBLER_FF1_Pin | DRIBBLER_FF2_Pin,
                      GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOG,
                      WHEEL_FRONT_LEFT_RESET_Pin | WHEEL_FRONT_LEFT_FF1_Pin |
                          WHEEL_FRONT_LEFT_FF2_Pin | WHEEL_BACK_LEFT_RESET_Pin |
                          WHEEL_BACK_LEFT_FF1_Pin | WHEEL_BACK_LEFT_FF2_Pin |
                          GENEVA_END_STOP_2_Pin | GENEVA_END_STOP_1_Pin | GENEVA_DIR_Pin,
                      GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(
        GPIOA, WHEEL_FRONT_RIGHT_DIR_Pin | DRIBBLER_DIR_Pin | WHEEL_BACK_LEFT_DIR_Pin,
        GPIO_PIN_RESET);

    /*Configure GPIO pins : PEPin PEPin PEPin PEPin
                             PEPin PEPin PEPin */
    GPIO_InitStruct.Pin = CHARGE_PWR_BRD_Pin | WHEEL_FRONT_RIGHT_FF1_Pin | IMU_INT1_Pin |
                          USER_LED_2_Pin | USER_LED_3_Pin | WHEEL_BACK_RIGHT_DIR_Pin |
                          WHEEL_BACK_RIGHT_FF1_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pins : PEPin PEPin PEPin PEPin */
    GPIO_InitStruct.Pin =
        FAULT_PWR_BRD_Pin | DONE_PWR_BRD_Pin | CH_SEL_3_Pin | CH_SEL_4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin   = IMU_INT2_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(IMU_INT2_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PFPin PFPin PFPin PFPin
                             PFPin */
    GPIO_InitStruct.Pin = WHEEL_FRONT_RIGHT_RESET_Pin | WHEEL_FRONT_RIGHT_FF2_Pin |
                          RED_IN_RGB_Pin | BLUE_IN_RGB_Pin | GREEN_IN_RGB_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin   = WHEEL_BACK_RIGHT_RESET_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(WHEEL_BACK_RIGHT_RESET_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PGPin PGPin PGPin */
    GPIO_InitStruct.Pin  = CH_SEL_1_Pin | CH_SEL_2_Pin | BREAKBEAM_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /*Configure GPIO pins : PDPin PDPin PDPin PDPin
                             PDPin */
    GPIO_InitStruct.Pin = WHEEL_BACK_RIGHT_FF2_Pin | ID_SEL_1_Pin | ID_SEL_3_Pin |
                          ID_SEL_2_Pin | ID_SEL_4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pins : PDPin PDPin PDPin PDPin
                             PDPin */
    GPIO_InitStruct.Pin = DRIBBLER_RESET_Pin | WHEEL_FRONT_LEFT_MODE_Pin |
                          WHEEL_FRONT_LEFT_DIR_Pin | DRIBBLER_FF1_Pin | DRIBBLER_FF2_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pins : PGPin PGPin PGPin PGPin
                             PGPin PGPin PGPin PGPin
                             PGPin */
    GPIO_InitStruct.Pin = WHEEL_FRONT_LEFT_RESET_Pin | WHEEL_FRONT_LEFT_FF1_Pin |
                          WHEEL_FRONT_LEFT_FF2_Pin | WHEEL_BACK_LEFT_RESET_Pin |
                          WHEEL_BACK_LEFT_FF1_Pin | WHEEL_BACK_LEFT_FF2_Pin |
                          GENEVA_END_STOP_2_Pin | GENEVA_END_STOP_1_Pin | GENEVA_DIR_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /*Configure GPIO pins : PCPin PCPin PCPin PCPin
                             PCPin */
    GPIO_InitStruct.Pin =
        SD_DAT0_Pin | SD_DAT1_Pin | SD_DAT2_Pin | SD_DAT3_Pin | SD_CLK_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : PAPin PAPin PAPin */
    GPIO_InitStruct.Pin =
        WHEEL_FRONT_RIGHT_DIR_Pin | DRIBBLER_DIR_Pin | WHEEL_BACK_LEFT_DIR_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin       = SD_CMD_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO1;
    HAL_GPIO_Init(SD_CMD_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
