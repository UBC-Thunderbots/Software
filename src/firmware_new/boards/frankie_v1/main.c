/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "external/nanopb/pb_decode.h"
#include "external/nanopb/pb_encode.h"
#include "firmware_new/proto/control.pb.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define DMA_BUFFER __attribute__((section(".dma_buffer")))
#define TX_BUFFER_LENGTH (1024)
#define RX_BUFFER_LENGTH (1024)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined(__ICCARM__) /*!< IAR Compiler */

#pragma location = 0x30040000
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location = 0x30040060
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
#pragma location = 0x30040200
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffers */

#elif defined(__CC_ARM) /* MDK ARM Compiler */

__attribute__((at(0x30040000)))
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30040060)))
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
__attribute__((at(0x30040200)))
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffer */

#elif defined(__GNUC__) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT]
    __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT]
    __attribute__((section(".TxDecripSection"))); /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]
    __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */

#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
// These buffers are stored in the R2 domain which is what the DMA
// controller has access to write to
DMA_BUFFER uint8_t recv_buf[RX_BUFFER_LENGTH];
DMA_BUFFER uint8_t send_buf[TX_BUFFER_LENGTH];

// This value is set in the idle line interrupt with the number of
// bytes left to transfer into the DMA buffer before it wraps around
volatile uint32_t dma_counter_on_idle = RX_BUFFER_LENGTH;

// Stores the last parsed byte position
volatile uint32_t last_byte_parsed = 0;

// global proto msg that will be updated to the most recent msg sent
control_msg control = control_msg_init_zero;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_DMA_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */


    /* Enable I-Cache---------------------------------------------------------*/
    SCB_EnableICache();

    /* MCU Configuration--------------------------------------------------------*/

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
    MX_ETH_Init();
    MX_USART3_UART_Init();
    MX_USB_OTG_FS_PCD_Init();
    MX_DMA_Init();
    /* USER CODE BEGIN 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    /* If we don't call these two functions (DeInit then Init) in this sequence,
     * we are only able to do one transfer and then everything grinds to a halt.
     * This was determined experimentally 
     * */

    HAL_UART_DeInit(&huart3);
    HAL_UART_Init(&huart3);

    /* We use idle line detection to know when to parse the circular buffer */
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    /* Setup DMA transfer to continually receive data over UART as the DMA
     * controller is setup in circular mode for rx/tx
     * NOTE: Even though this is in a while loop, it only takes 1 or 2 tries
     * for HAL to not be busy and initialize the DMA transfer  */
    while (HAL_UART_Receive_DMA(&huart3, recv_buf, RX_BUFFER_LENGTH) != HAL_OK) {}

    while (1)
    {
        // if the dma_counter is at a new position, then we parse the data
        // NOTE that we subtract the buffer length as the dma counter is the number
        // of bytes left to transfer, we would like the number of bytes transfered
        if ((RX_BUFFER_LENGTH - dma_counter_on_idle) != last_byte_parsed) {
            parse_control_msg_from_dma_buffer(recv_buf, RX_BUFFER_LENGTH, last_byte_parsed, dma_counter_on_idle);
            last_byte_parsed = RX_BUFFER_LENGTH - dma_counter_on_idle;
        }
        /* USER CODE END 2 */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct         = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct         = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    /** Supply configuration update enable
     */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    /** Configure the main internal regulator output voltage
     */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
    {
    }
    /** Initializes the CPU, AHB and APB busses clocks
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 1;
    RCC_OscInitStruct.PLL.PLLN       = 24;
    RCC_OscInitStruct.PLL.PLLP       = 2;
    RCC_OscInitStruct.PLL.PLLQ       = 4;
    RCC_OscInitStruct.PLL.PLLR       = 2;
    RCC_OscInitStruct.PLL.PLLRGE     = RCC_PLL1VCIRANGE_3;
    RCC_OscInitStruct.PLL.PLLVCOSEL  = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN   = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                  RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_USB;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    PeriphClkInitStruct.UsbClockSelection         = RCC_USBCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Enable USB Voltage detector
     */
    HAL_PWREx_EnableUSBVoltageDetector();
}

/**
 * @brief ETH Initialization Function
 * @param None
 * @retval None
 */
static void MX_ETH_Init(void)
{
    /* USER CODE BEGIN ETH_Init 0 */

    /* USER CODE END ETH_Init 0 */

    /* USER CODE BEGIN ETH_Init 1 */

    /* USER CODE END ETH_Init 1 */
    heth.Instance            = ETH;
    heth.Init.MACAddr[0]     = 0x00;
    heth.Init.MACAddr[1]     = 0x80;
    heth.Init.MACAddr[2]     = 0xE1;
    heth.Init.MACAddr[3]     = 0x00;
    heth.Init.MACAddr[4]     = 0x00;
    heth.Init.MACAddr[5]     = 0x00;
    heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
    heth.Init.TxDesc         = DMATxDscrTab;
    heth.Init.RxDesc         = DMARxDscrTab;
    heth.Init.RxBuffLen      = 1524;

    /* USER CODE BEGIN MACADDRESS */

    /* USER CODE END MACADDRESS */

    if (HAL_ETH_Init(&heth) != HAL_OK)
    {
        Error_Handler();
    }

    memset(&TxConfig, 0, sizeof(ETH_TxPacketConfig));
    TxConfig.Attributes   = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
    TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
    TxConfig.CRCPadCtrl   = ETH_CRC_PAD_INSERT;
    /* USER CODE BEGIN ETH_Init 2 */

    /* USER CODE END ETH_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{
    /* USER CODE BEGIN USART3_Init 0 */

    /* USER CODE END USART3_Init 0 */

    /* USER CODE BEGIN USART3_Init 1 */

    /* USER CODE END USART3_Init 1 */
    huart3.Instance                    = USART3;
    huart3.Init.BaudRate               = 115200;
    huart3.Init.WordLength             = UART_WORDLENGTH_8B;
    huart3.Init.StopBits               = UART_STOPBITS_1;
    huart3.Init.Parity                 = UART_PARITY_NONE;
    huart3.Init.Mode                   = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART3_Init 2 */

    /* USER CODE END USART3_Init 2 */
}

/**
 * @brief USB_OTG_FS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_FS_PCD_Init(void)
{
    /* USER CODE BEGIN USB_OTG_FS_Init 0 */

    /* USER CODE END USB_OTG_FS_Init 0 */

    /* USER CODE BEGIN USB_OTG_FS_Init 1 */

    /* USER CODE END USB_OTG_FS_Init 1 */
    hpcd_USB_OTG_FS.Instance                     = USB_OTG_FS;
    hpcd_USB_OTG_FS.Init.dev_endpoints           = 9;
    hpcd_USB_OTG_FS.Init.speed                   = PCD_SPEED_FULL;
    hpcd_USB_OTG_FS.Init.dma_enable              = DISABLE;
    hpcd_USB_OTG_FS.Init.phy_itface              = PCD_PHY_EMBEDDED;
    hpcd_USB_OTG_FS.Init.Sof_enable              = ENABLE;
    hpcd_USB_OTG_FS.Init.low_power_enable        = DISABLE;
    hpcd_USB_OTG_FS.Init.lpm_enable              = DISABLE;
    hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
    hpcd_USB_OTG_FS.Init.vbus_sensing_enable     = ENABLE;
    hpcd_USB_OTG_FS.Init.use_dedicated_ep1       = DISABLE;
    if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USB_OTG_FS_Init 2 */

    /* USER CODE END USB_OTG_FS_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Stream6_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    /* DMA1_Stream7_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, LD3_Pin | LD2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : USER_Btn_Pin */
    GPIO_InitStruct.Pin  = USER_Btn_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : LD3_Pin LD2_Pin */
    GPIO_InitStruct.Pin   = LD3_Pin | LD2_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
    GPIO_InitStruct.Pin   = USB_PowerSwitchOn_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : USB_OverCurrent_Pin */
    GPIO_InitStruct.Pin  = USB_OverCurrent_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/**
 * @brief  Updates a control msg struct with the given dma buffer, start and end positions
 *
 *  Example: Consider the following circular buffer, notice that byte 13, 14, 15 wrap
 *  around back to the start of the buffer.
 *
 *  BUFFER = [ b13 | b14 | b15 | b4 | b5 | b6 | b7 | b8 | b9 | b10 | b11 | b12 ]
 *                         dci                                 lbp
 *
 *  Here, lbp (last byte parsed) is the last position of the last msg that was received 
 *  and decoded. dci (dma_counter_on_idle) is the new position where we detected an idle line,
 *  indicating the end of another transfer.
 *
 *  NOTE: both dci and lbp must be less than buffer size
 *  NOTE: we also make the assumption that the buffer is large enough that we
 *  don't wrap around more than once for each msg
 *  NOTE: this function modifies the global control_msg (control)
 *
 * @param  rx_buf: pointer to the base of the DMA buffer
 * @param  size: the max size of the circular DMA buffer, used to wrap around
 * @param  last_byte_parsed: The position of the last byte that was parsed
 * @param  dma_counter_on_idle: The position of the last byte that was coun
 * @retval -1 for error, 0 otherwise
 */
int parse_control_msg_from_dma_buffer(
        uint8_t* rx_buf, uint32_t size, uint32_t last_byte_parsed, uint32_t dma_counter_on_idle) {

    // the dma_counter when we detected an IDLE line is less than the position of
    // the previously parsed byte, we have wrapped around the buffer
    //
    // NOTE: the dma counter will be how many bytes are left in the buffer, we have to subtract
    // the size to get the position from the beginning of the buffer
    uint32_t buffer_position = size - dma_counter_on_idle;

    if (buffer_position <= last_byte_parsed) {

        uint32_t msg_size = (size - last_byte_parsed) + buffer_position;
        uint8_t buffer_to_parse[msg_size];

        uint32_t index = 0;
        for (uint32_t k = last_byte_parsed; k < size; k++) {
            buffer_to_parse[index++] = rx_buf[k];
        }

        for (uint32_t k = 0; k < buffer_position; k++) {
            buffer_to_parse[index++] = rx_buf[k];
        }

        // Create a stream that reads from the buffer. 
        pb_istream_t in_stream = pb_istream_from_buffer(buffer_to_parse, msg_size);

        // TODO error check
        if (pb_decode(&in_stream, control_msg_fields, &control))
        {
            return 0;
        }

    // the buffer has not wrapped around yet
    } else if (buffer_position > last_byte_parsed) {

        uint32_t msg_size = buffer_position - last_byte_parsed;
        uint8_t buffer_to_parse[msg_size];

        uint32_t index = 0;
        for (uint32_t k = last_byte_parsed; k <= buffer_position; k++) {
            buffer_to_parse[index++] = rx_buf[k];
        }

        // Create a stream that reads from the buffer. 
        pb_istream_t in_stream = pb_istream_from_buffer(buffer_to_parse, msg_size);

        // TODO error check
        if (pb_decode(&in_stream, control_msg_fields, &control))
        {
            return 0;
        }
    }

    return -1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    return;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    return;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    Error_Handler();
}

void USER_UART_IdleLineCallback(UART_HandleTypeDef *huart)
{
    dma_counter_on_idle = __HAL_DMA_GET_COUNTER(huart->hdmarx);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
