/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "firmware/motor/main.h"

#include "firmware/motor/crc.h"
#include "firmware/motor/firmware.h"
#include "firmware/motor/mc_api.h"
#include "firmware/motor/mc_interface.h"
#include "firmware/motor/stm32f0xx/legacy/stm32_hal_legacy.h"
#include "firmware/motor/stm32f0xx/stm32f031x6.h"
#include "firmware/motor/stm32f0xx/stm32f0xx_hal_spi.h"
#include "firmware/motor/types.h"

#include <stdint.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
SPI_HandleTypeDef hspi1;

int16_t ax = 0, bx = 0;
uint8_t TX_Buffer[FRAME_SIZE] = {0};
uint8_t RX_Buffer[FRAME_SIZE] = {0};

volatile uint8_t new_data_received = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
static void MX_SPI1_Init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration --------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    // MX_USART1_UART_Init();
    MX_MotorControl_Init();

    /* Initialize interrupts */
    MX_NVIC_Init();
    /* USER CODE BEGIN 2 */
    MX_SPI1_Init();

    // call it once for live expression
    // MC_GetSTMStateMotor1();  // set a breakpoint on the line if reading the
    //                          // state via Debugger
    // MC_GetOccurredFaultsMotor1();

    // Initial arm of SPI recv
    if (HAL_SPI_TransmitReceive_IT(&hspi1, TX_Buffer, RX_Buffer, FRAME_SIZE) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */
        /* USER CODE BEGIN 3 */
        // TODO: Need 2 add double buffering to prevent race condition
        if (new_data_received) {
            // frame integrity check
            if (RX_Buffer[0] != FRAME_SOF || RX_Buffer[5] != FRAME_EOF ||
                RX_Buffer[4] != crc_gen_checksum(RX_Buffer[1], (RX_Buffer[2] << 8) + RX_Buffer[3])) {
                TX_Buffer[3] = 0;
                // send the NACK
                new_data_received = false;
                continue;
            }

            uint16_t data = 0;

            switch (RX_Buffer[1]) {
                case MOV_AX:
                    ax = (RX_Buffer[2] << 8) + RX_Buffer[3];
                    break;
                case GET_AX:
                    data = ax;
                    break;
                case MOV_BX:
                    bx = (RX_Buffer[2] << 8) + RX_Buffer[3];
                    break;
                case GET_BX:
                    data = bx;
                    break;
                case SET_SPEEDRAMP:
                    MC_ProgramSpeedRampMotor1(ax, bx);
                    break;
                case GET_SPEED:
                    data = MC_GetMecSpeedReferenceMotor1();
                    break;
                case GET_ENCODER:
                    // TODO: Implement GET_ENCODER FUNCTION
                    // ax = MC_
                    break;
                case START_MOTOR:
                    MC_StartMotor1();
                    break;
                case STOP_MOTOR:
                    MC_StopMotor1();
                    break;
                case ACK_FAULTS:
                    MC_AcknowledgeFaultMotor1();
                    break;
                case GET_FAULT:
                    data = MC_GetOccurredFaultsMotor1();
                    break;
                case SET_CURRENT:
                case GET_CURRENT:
                default:
                    break;
            }

            uint8_t checksum = crc_gen_checksum(ACK, data);
            TX_Buffer[0]     = FRAME_SOF;
            TX_Buffer[1]     = ACK;
            TX_Buffer[2]     = (data >> 8) & 0xFF;
            TX_Buffer[3]     = data & 0xFF;
            TX_Buffer[4]     = checksum;
            TX_Buffer[5]     = FRAME_EOF;

            new_data_received = 0;
        }
    }
    /* USER CODE END 3 */
}

/**
 * @brief  TxRx Transfer completed callback.
 * @param  hspi: pointer to a SPI_HandleTypeDef structure.
 * @retval None
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
    // let main thread know new data has been recv
    if (hspi->Instance == SPI1) {
        new_data_received = 1;
    }

    // rearm the receiver
    if (HAL_SPI_TransmitReceive_IT(&hspi1, TX_Buffer, RX_Buffer, FRAME_SIZE) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief  SPI error callback.
 * @param  hspi: pointer to a SPI_HandleTypeDef structure.
 * @retval None
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi) {
    if (hspi->Instance == SPI1) {
        Error_Handler();
    }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1) {
    }
    LL_RCC_HSE_Enable();

    /* Wait till HSE is ready */
    while (LL_RCC_HSE_IsReady() != 1) {
    }
    LL_RCC_HSI14_Enable();

    /* Wait till HSI14 is ready */
    while (LL_RCC_HSI14_IsReady() != 1) {
    }
    LL_RCC_HSI14_SetCalibTrimming(16);
    LL_RCC_HSE_EnableCSS();
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_6);
    LL_RCC_PLL_Enable();

    /* Wait till PLL is ready */
    while (LL_RCC_PLL_IsReady() != 1) {
    }
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    /* Wait till System clock is ready */
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
    }
    LL_SetSystemCoreClock(48000000);

    /* Update the time base */
    if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK) {
        Error_Handler();
    }
    LL_RCC_HSI14_EnableADCControl();
    LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void) {
    /* USART1_IRQn interrupt configuration */
    // NVIC_SetPriority(USART1_IRQn, 3);
    // NVIC_EnableIRQ(USART1_IRQn);
    /* SPI1_IRQn interrupt configuration */
    NVIC_SetPriority(SPI1_IRQn, 3);
    NVIC_EnableIRQ(SPI1_IRQn);
    /* DMA1_Channel1_IRQn interrupt configuration */
    NVIC_SetPriority(DMA1_Channel1_IRQn, 1);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    /* DMA1_Channel4_5_IRQn interrupt configuration */
    NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0);
    NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);
    /* DMA1_Channel2_3_IRQn interrupt configuration */
    NVIC_SetPriority(DMA1_Channel2_3_IRQn, 3);
    NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
    /* TIM1_BRK_UP_TRG_COM_IRQn interrupt configuration */
    NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 0);
    NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
    /* TIM2_IRQn interrupt configuration */
    NVIC_SetPriority(TIM2_IRQn, 3);
    NVIC_EnableIRQ(TIM2_IRQn);
}

/**
 * @brief ADC Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC_Init(void) {
    /* USER CODE BEGIN ADC_Init 0 */

    /* USER CODE END ADC_Init 0 */

    LL_ADC_InitTypeDef ADC_InitStruct         = {0};
    LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    /**ADC GPIO Configuration
    PA5   ------> ADC_IN5
    */
    GPIO_InitStruct.Pin  = M1_CURR_AMPL_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(M1_CURR_AMPL_GPIO_Port, &GPIO_InitStruct);

    /* ADC DMA Init */

    /* ADC Init */
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);

    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);

    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);

    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);

    /* USER CODE BEGIN ADC_Init 1 */

    /* USER CODE END ADC_Init 1 */

    /** Configure Regular Channel
     */
    LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_5);

    /** Configure the global features of the ADC (Clock, Resolution, Data
     * Alignment and number of conversion)
     */
    ADC_InitStruct.Clock         = LL_ADC_CLOCK_ASYNC;
    ADC_InitStruct.Resolution    = LL_ADC_RESOLUTION_12B;
    ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_LEFT;
    ADC_InitStruct.LowPowerMode  = LL_ADC_LP_MODE_NONE;
    LL_ADC_Init(ADC1, &ADC_InitStruct);
    ADC_REG_InitStruct.TriggerSource    = LL_ADC_REG_TRIG_EXT_TIM1_TRGO;
    ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
    ADC_REG_InitStruct.ContinuousMode   = LL_ADC_REG_CONV_SINGLE;
    ADC_REG_InitStruct.DMATransfer      = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
    ADC_REG_InitStruct.Overrun          = LL_ADC_REG_OVR_DATA_PRESERVED;
    LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
    LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
    LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_7CYCLES_5);
    LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
    /* USER CODE BEGIN ADC_Init 2 */

    /* USER CODE END ADC_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {
    /* USER CODE BEGIN TIM1_Init 0 */

    /* USER CODE END TIM1_Init 0 */

    LL_TIM_InitTypeDef TIM_InitStruct          = {0};
    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct    = {0};
    LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    /**TIM1 GPIO Configuration
    PB12   ------> TIM1_BKIN
    */
    GPIO_InitStruct.Pin        = M1_OCP_Pin;
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate  = LL_GPIO_AF_2;
    LL_GPIO_Init(M1_OCP_GPIO_Port, &GPIO_InitStruct);

    /* TIM1 DMA Init */

    /* TIM1_CH4_TRIG_COM Init */
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PRIORITY_HIGH);

    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MODE_CIRCULAR);

    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PERIPH_NOINCREMENT);

    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MEMORY_INCREMENT);

    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PDATAALIGN_HALFWORD);

    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MDATAALIGN_HALFWORD);

    /* TIM1_CH3_UP Init */
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_5, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PRIORITY_HIGH);

    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MODE_CIRCULAR);

    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PERIPH_NOINCREMENT);

    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MEMORY_INCREMENT);

    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PDATAALIGN_HALFWORD);

    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MDATAALIGN_HALFWORD);

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    TIM_InitStruct.Prescaler         = ((TIM_CLOCK_DIVIDER)-1);
    TIM_InitStruct.CounterMode       = LL_TIM_COUNTERMODE_CENTER_UP;
    TIM_InitStruct.Autoreload        = ((PWM_PERIOD_CYCLES) / 2);
    TIM_InitStruct.ClockDivision     = LL_TIM_CLOCKDIVISION_DIV2;
    TIM_InitStruct.RepetitionCounter = (REP_COUNTER);
    LL_TIM_Init(TIM1, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM1);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
    TIM_OC_InitStruct.OCMode       = LL_TIM_OCMODE_PWM1;
    TIM_OC_InitStruct.OCState      = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState     = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.CompareValue = ((PWM_PERIOD_CYCLES) / 4);
    TIM_OC_InitStruct.OCPolarity   = LL_TIM_OCPOLARITY_LOW;
    TIM_OC_InitStruct.OCNPolarity  = LL_TIM_OCPOLARITY_LOW;
    TIM_OC_InitStruct.OCIdleState  = LL_TIM_OCIDLESTATE_HIGH;
    TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_HIGH;
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH3);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH4);
    TIM_OC_InitStruct.OCMode       = LL_TIM_OCMODE_PWM2;
    TIM_OC_InitStruct.CompareValue = (((PWM_PERIOD_CYCLES) / 2) - (HTMIN));
    TIM_OC_InitStruct.OCPolarity   = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCIdleState  = LL_TIM_OCIDLESTATE_LOW;
    TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH4);
    LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_UPDATE);
    LL_TIM_DisableMasterSlaveMode(TIM1);
    TIM_BDTRInitStruct.OSSRState       = LL_TIM_OSSR_ENABLE;
    TIM_BDTRInitStruct.OSSIState       = LL_TIM_OSSI_ENABLE;
    TIM_BDTRInitStruct.LockLevel       = LL_TIM_LOCKLEVEL_OFF;
    TIM_BDTRInitStruct.DeadTime        = ((DEAD_TIME_COUNTS) / 2);
    TIM_BDTRInitStruct.BreakState      = LL_TIM_BREAK_ENABLE;
    TIM_BDTRInitStruct.BreakPolarity   = LL_TIM_BREAK_POLARITY_LOW;
    TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
    LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);
    /* USER CODE BEGIN TIM1_Init 2 */

    /* USER CODE END TIM1_Init 2 */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    /**TIM1 GPIO Configuration
    PB13   ------> TIM1_CH1N
    PB14   ------> TIM1_CH2N
    PB15   ------> TIM1_CH3N
    PA8   ------> TIM1_CH1
    PA9   ------> TIM1_CH2
    PA10   ------> TIM1_CH3
    */
    GPIO_InitStruct.Pin        = M1_PWM_UL_Pin;
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate  = LL_GPIO_AF_2;
    LL_GPIO_Init(M1_PWM_UL_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin        = M1_PWM_VL_Pin;
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate  = LL_GPIO_AF_2;
    LL_GPIO_Init(M1_PWM_VL_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin        = M1_PWM_WL_Pin;
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate  = LL_GPIO_AF_2;
    LL_GPIO_Init(M1_PWM_WL_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin        = M1_PWM_UH_Pin;
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate  = LL_GPIO_AF_2;
    LL_GPIO_Init(M1_PWM_UH_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin        = M1_PWM_VH_Pin;
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate  = LL_GPIO_AF_2;
    LL_GPIO_Init(M1_PWM_VH_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin        = M1_PWM_WH_Pin;
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate  = LL_GPIO_AF_2;
    LL_GPIO_Init(M1_PWM_WH_GPIO_Port, &GPIO_InitStruct);
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {
    /* USER CODE BEGIN TIM2_Init 0 */

    /* USER CODE END TIM2_Init 0 */

    LL_TIM_InitTypeDef TIM_InitStruct       = {0};
    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    /**TIM2 GPIO Configuration
    PA0   ------> TIM2_CH1
    PA1   ------> TIM2_CH2
    PA2   ------> TIM2_CH3
    */
    GPIO_InitStruct.Pin        = M1_HALL_H1_Pin;
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate  = LL_GPIO_AF_2;
    LL_GPIO_Init(M1_HALL_H1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin        = M1_HALL_H2_Pin;
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate  = LL_GPIO_AF_2;
    LL_GPIO_Init(M1_HALL_H2_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin        = M1_HALL_H3_Pin;
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate  = LL_GPIO_AF_2;
    LL_GPIO_Init(M1_HALL_H3_GPIO_Port, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    TIM_InitStruct.Prescaler     = 0;
    TIM_InitStruct.CounterMode   = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload    = M1_HALL_TIM_PERIOD;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    LL_TIM_Init(TIM2, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM2);
    LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
    LL_TIM_CC_DisableChannel(TIM2, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
    LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_TRC);
    LL_TIM_IC_SetPrescaler(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH1, M1_HALL_IC_FILTER);
    LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
    LL_TIM_IC_EnableXORCombination(TIM2);
    LL_TIM_SetTriggerInput(TIM2, LL_TIM_TS_TI1F_ED);
    LL_TIM_SetSlaveMode(TIM2, LL_TIM_SLAVEMODE_RESET);
    TIM_OC_InitStruct.OCMode       = LL_TIM_OCMODE_PWM2;
    TIM_OC_InitStruct.OCState      = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState     = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.CompareValue = 0;
    TIM_OC_InitStruct.OCPolarity   = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCNPolarity  = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCIdleState  = LL_TIM_OCIDLESTATE_LOW;
    TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;

    LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH2);
    LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_OC2REF);
    LL_TIM_DisableMasterSlaveMode(TIM2);
    /* USER CODE BEGIN TIM2_Init 2 */

    /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {
    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    LL_USART_InitTypeDef USART_InitStruct = {0};

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    /**USART1 GPIO Configuration
    PB6   ------> USART1_TX
    PB7   ------> USART1_RX
    */
    GPIO_InitStruct.Pin        = UART_TX_Pin;
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate  = LL_GPIO_AF_0;
    LL_GPIO_Init(UART_TX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin        = UART_RX_Pin;
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate  = LL_GPIO_AF_0;
    LL_GPIO_Init(UART_RX_GPIO_Port, &GPIO_InitStruct);

    /* USART1 DMA Init */

    /* USART1_RX Init */
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_LOW);

    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_NORMAL);

    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);

    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);

    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_BYTE);

    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_BYTE);

    /* USART1_TX Init */
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);

    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);

    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);

    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);

    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);

    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    USART_InitStruct.BaudRate            = 1843200;
    USART_InitStruct.DataWidth           = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits            = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity              = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection   = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling        = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART1, &USART_InitStruct);
    LL_USART_DisableIT_CTS(USART1);
    LL_USART_ConfigAsyncMode(USART1);
    LL_USART_Enable(USART1);
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
    /* Init with LL driver */
    /* DMA controller clock enable */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

    /**/
    LL_GPIO_SetOutputPin(M1_EN_DRIVER_GPIO_Port, M1_EN_DRIVER_Pin);

    /**/
    GPIO_InitStruct.Pin  = LL_GPIO_PIN_7;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin  = LL_GPIO_PIN_0;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin        = M1_EN_DRIVER_Pin;
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_UP;
    LL_GPIO_Init(M1_EN_DRIVER_GPIO_Port, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void MX_SPI1_Init(void) {
    hspi1.Instance            = SPI1;
    hspi1.Init.Mode           = SPI_MODE_SLAVE;
    hspi1.Init.Direction      = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize       = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity    = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase       = SPI_PHASE_1EDGE;
    hspi1.Init.NSS            = SPI_NSS_HARD_INPUT;
    hspi1.Init.FirstBit       = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode         = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial  = 7;
    hspi1.Init.CRCLength      = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode       = SPI_NSS_PULSE_DISABLE;

    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state
     */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, ex printf("Wrong parameters value: file %s on line %d\r\n", file,
       line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
