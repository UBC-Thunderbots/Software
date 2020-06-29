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
 */ /* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "cmsis_os.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "firmware_new/boards/frankie_v1/io/drivetrain.h"
#include "firmware_new/boards/frankie_v1/io/proto_multicast_communication_profile.h"
#include "firmware_new/boards/frankie_v1/io/proto_multicast_communication_tasks.h"
#include "shared/constants.h"
#include "shared/proto/tbots_robot_msg.pb.h"
#include "shared/proto/tbots_software_msgs.pb.h"

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

CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for NetStartTask */
osThreadId_t NetStartTaskHandle;
const osThreadAttr_t NetStartTask_attributes = {.name     = "NetStartTask",
                                                .priority = (osPriority_t)osPriorityHigh7,
                                                .stack_size = 1024 * 4};
/* Definitions for RobotStatusTask */
osThreadId_t RobotStatusTaskHandle;
const osThreadAttr_t RobotStatusTask_attributes = {
    .name       = "RobotStatusTask",
    .priority   = (osPriority_t)osPriorityHigh7,
    .stack_size = 1024 * 4};
/* Definitions for VisionMsgTask */
osThreadId_t VisionMsgTaskHandle;
const osThreadAttr_t VisionMsgTask_attributes = {
    .name       = "VisionMsgTask",
    .priority   = (osPriority_t)osPriorityHigh7,
    .stack_size = 1024 * 4};
/* Definitions for PrimMsgTask */
osThreadId_t PrimMsgTaskHandle;
const osThreadAttr_t PrimMsgTask_attributes = {.name     = "PrimMsgTask",
                                               .priority = (osPriority_t)osPriorityHigh7,
                                               .stack_size = 1024 * 4};
/* Definitions for testMsgUpdate */
osThreadId_t testMsgUpdateHandle;
const osThreadAttr_t testMsgUpdate_attributes = {
    .name       = "testMsgUpdate",
    .priority   = (osPriority_t)osPriorityNormal1,
    .stack_size = 1024 * 4};
/* USER CODE BEGIN PV */

ProtoMulticastCommunicationProfile_t *tbots_robot_msg_sender_profile;
ProtoMulticastCommunicationProfile_t *vision_msg_listener_profile;
ProtoMulticastCommunicationProfile_t *primitive_msg_listener_profile;

static VisionMsg vision_msg;
static TbotsRobotMsg tbots_robot_msg;
static PrimitiveMsg primitive_msg;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM4_Init(void);
void io_proto_multicast_startNetworkingTask(void *argument);
extern void io_proto_multicast_sender_task(void *argument);
extern void io_proto_multicast_listener_task(void *argument);
void test_msg_update(void *argument);

/* USER CODE BEGIN PFP */

static void initIoLayer(void);
static void initIoDrivetrain(void);
static void initIoNetworking(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void initIoLayer(void)
{
    initIoDrivetrain();
    initIoNetworking();
}

void initIoDrivetrain(void)
{
    // Initialize a motor driver with the given suffix, on the given
    // timer channel
#define INIT_DRIVETRAIN_UNIT(MOTOR_NAME_SUFFIX, TIMER_CHANNEL)                           \
    {                                                                                    \
        GpioPin_t *reset_pin =                                                           \
            io_gpio_pin_create(wheel_motor_##MOTOR_NAME_SUFFIX##_reset_GPIO_Port,        \
                               wheel_motor_##MOTOR_NAME_SUFFIX##_reset_Pin, ACTIVE_LOW); \
        GpioPin_t *coast_pin =                                                           \
            io_gpio_pin_create(wheel_motor_##MOTOR_NAME_SUFFIX##_coast_GPIO_Port,        \
                               wheel_motor_##MOTOR_NAME_SUFFIX##_coast_Pin, ACTIVE_LOW); \
        GpioPin_t *mode_pin =                                                            \
            io_gpio_pin_create(wheel_motor_##MOTOR_NAME_SUFFIX##_mode_GPIO_Port,         \
                               wheel_motor_##MOTOR_NAME_SUFFIX##_mode_Pin, ACTIVE_HIGH); \
        GpioPin_t *direction_pin = io_gpio_pin_create(                                   \
            wheel_motor_##MOTOR_NAME_SUFFIX##_direction_GPIO_Port,                       \
            wheel_motor_##MOTOR_NAME_SUFFIX##_direction_Pin, ACTIVE_HIGH);               \
        GpioPin_t *brake_pin =                                                           \
            io_gpio_pin_create(wheel_motor_##MOTOR_NAME_SUFFIX##_brake_GPIO_Port,        \
                               wheel_motor_##MOTOR_NAME_SUFFIX##_brake_Pin, ACTIVE_LOW); \
        GpioPin_t *esf_pin =                                                             \
            io_gpio_pin_create(wheel_motor_##MOTOR_NAME_SUFFIX##_esf_GPIO_Port,          \
                               wheel_motor_##MOTOR_NAME_SUFFIX##_esf_Pin, ACTIVE_HIGH);  \
        PwmPin_t *pwm_pin = io_pwm_pin_create(&htim4, TIMER_CHANNEL);                    \
                                                                                         \
        AllegroA3931MotorDriver_t *motor_driver = io_allegro_a3931_motor_driver_create(  \
            pwm_pin, reset_pin, coast_pin, mode_pin, direction_pin, brake_pin, esf_pin); \
        io_allegro_a3931_motor_setPwmPercentage(motor_driver, 0.0);                      \
        drivetrain_unit_##MOTOR_NAME_SUFFIX = io_drivetrain_unit_create(motor_driver);   \
    }

    DrivetrainUnit_t *drivetrain_unit_front_left;
    DrivetrainUnit_t *drivetrain_unit_back_left;
    DrivetrainUnit_t *drivetrain_unit_back_right;
    DrivetrainUnit_t *drivetrain_unit_front_right;
    INIT_DRIVETRAIN_UNIT(front_left, TIM_CHANNEL_1);
    INIT_DRIVETRAIN_UNIT(back_left, TIM_CHANNEL_2);
    INIT_DRIVETRAIN_UNIT(back_right, TIM_CHANNEL_3);
    INIT_DRIVETRAIN_UNIT(front_right, TIM_CHANNEL_4);

    io_drivetrain_init(drivetrain_unit_front_left, drivetrain_unit_front_right,
                       drivetrain_unit_back_left, drivetrain_unit_back_right);
}

void initIoNetworking()
{
    // TODO this needs to be hooked up to the channel dial on the robot, when available
    // https://github.com/UBC-Thunderbots/Software/issues/1517
    unsigned channel = 0;

    io_proto_multicast_communication_init(NETWORK_TIMEOUT_MS);

    primitive_msg_listener_profile = io_proto_multicast_communication_profile_create(
        "primitive_msg_listener_profile", MULTICAST_CHANNELS[channel], PRIMITIVE_PORT,
        &primitive_msg, PrimitiveMsg_fields, MAXIMUM_TRANSFER_UNIT_BYTES);

    vision_msg_listener_profile = io_proto_multicast_communication_profile_create(
        "vision_msg_listener_profile", MULTICAST_CHANNELS[channel], VISION_PORT,
        &vision_msg, VisionMsg_fields, MAXIMUM_TRANSFER_UNIT_BYTES);

    tbots_robot_msg_sender_profile = io_proto_multicast_communication_profile_create(
        "tbots_robot_msg_sender", MULTICAST_CHANNELS[channel], ROBOT_STATUS_PORT,
        &tbots_robot_msg, TbotsRobotMsg_fields, MAXIMUM_TRANSFER_UNIT_BYTES);
}


/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MPU Configuration--------------------------------------------------------*/
    MPU_Config();

    /* Enable I-Cache---------------------------------------------------------*/
    SCB_EnableICache();

    /* Enable D-Cache---------------------------------------------------------*/
    SCB_EnableDCache();

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
    MX_USART3_UART_Init();
    MX_USB_OTG_FS_PCD_Init();
    MX_CRC_Init();
    MX_TIM4_Init();
    /* USER CODE BEGIN 2 */

    initIoLayer();

    /* USER CODE END 2 */

    /* Init scheduler */
    osKernelInitialize();

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of NetStartTask */
    NetStartTaskHandle = osThreadNew(io_proto_multicast_startNetworkingTask, NULL,
                                     &NetStartTask_attributes);

    /* creation of RobotStatusTask */
    RobotStatusTaskHandle =
        osThreadNew(io_proto_multicast_sender_task,
                    (void *)tbots_robot_msg_sender_profile, &RobotStatusTask_attributes);

    /* creation of VisionMsgTask */
    VisionMsgTaskHandle =
        osThreadNew(io_proto_multicast_listener_task, (void *)vision_msg_listener_profile,
                    &VisionMsgTask_attributes);

    /* creation of PrimMsgTask */
    PrimMsgTaskHandle =
        osThreadNew(io_proto_multicast_listener_task,
                    (void *)primitive_msg_listener_profile, &PrimMsgTask_attributes);

    /* creation of testMsgUpdate */
    testMsgUpdateHandle =
        osThreadNew(test_msg_update, (void *)tbots_robot_msg_sender_profile,
                    &testMsgUpdate_attributes);

    /* USER CODE BEGIN RTOS_THREADS */

    /* USER CODE END RTOS_THREADS */

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

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
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void)
{
    /* USER CODE BEGIN CRC_Init 0 */

    /* USER CODE END CRC_Init 0 */

    /* USER CODE BEGIN CRC_Init 1 */

    /* USER CODE END CRC_Init 1 */
    hcrc.Instance                     = CRC;
    hcrc.Init.DefaultPolynomialUse    = DEFAULT_POLYNOMIAL_ENABLE;
    hcrc.Init.DefaultInitValueUse     = DEFAULT_INIT_VALUE_ENABLE;
    hcrc.Init.InputDataInversionMode  = CRC_INPUTDATA_INVERSION_NONE;
    hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
    hcrc.InputDataFormat              = CRC_INPUTDATA_FORMAT_BYTES;
    if (HAL_CRC_Init(&hcrc) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN CRC_Init 2 */

    /* USER CODE END CRC_Init 2 */
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{
    /* USER CODE BEGIN TIM4_Init 0 */

    /* USER CODE END TIM4_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC          = {0};

    /* USER CODE BEGIN TIM4_Init 1 */

    /* USER CODE END TIM4_Init 1 */
    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = 9 - 1;
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = 400 - 1;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM4_Init 2 */

    /* USER CODE END TIM4_Init 2 */
    HAL_TIM_MspPostInit(&htim4);
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
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(
        GPIOF,
        wheel_motor_back_right_esf_Pin | wheel_motor_front_right_reset_Pin |
            wheel_motor_front_right_coast_Pin | wheel_motor_front_right_mode_Pin |
            wheel_motor_front_right_direction_Pin | wheel_motor_front_right_brake_Pin |
            wheel_motor_front_right_esf_Pin,
        GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(
        GPIOC, wheel_motor_back_right_coast_Pin | wheel_motor_back_left_direction_Pin,
        GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA,
                      wheel_motor_back_right_brake_Pin |
                          wheel_motor_back_right_reset_Pin |
                          wheel_motor_back_right_direction_Pin,
                      GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(
        GPIOB,
        wheel_motor_back_left_brake_Pin | wheel_motor_back_left_esf_Pin |
            wheel_motor_front_left_esf_Pin | wheel_motor_back_left_reset_Pin |
            wheel_motor_back_left_coast_Pin | LD3_Pin | wheel_motor_back_left_mode_Pin |
            wheel_motor_front_left_reset_Pin | wheel_motor_front_left_coast_Pin |
            wheel_motor_front_left_mode_Pin | LD2_Pin |
            wheel_motor_front_left_direction_Pin | wheel_motor_front_left_brake_Pin,
        GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(wheel_motor_back_right_mode_GPIO_Port,
                      wheel_motor_back_right_mode_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : USER_Btn_Pin */
    GPIO_InitStruct.Pin  = USER_Btn_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : wheel_motor_back_right_esf_Pin
       wheel_motor_front_right_reset_Pin wheel_motor_front_right_coast_Pin
       wheel_motor_front_right_mode_Pin wheel_motor_front_right_direction_Pin
       wheel_motor_front_right_brake_Pin wheel_motor_front_right_esf_Pin */
    GPIO_InitStruct.Pin =
        wheel_motor_back_right_esf_Pin | wheel_motor_front_right_reset_Pin |
        wheel_motor_front_right_coast_Pin | wheel_motor_front_right_mode_Pin |
        wheel_motor_front_right_direction_Pin | wheel_motor_front_right_brake_Pin |
        wheel_motor_front_right_esf_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /*Configure GPIO pins : wheel_motor_back_right_coast_Pin
     * wheel_motor_back_left_direction_Pin */
    GPIO_InitStruct.Pin =
        wheel_motor_back_right_coast_Pin | wheel_motor_back_left_direction_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : wheel_motor_back_right_brake_Pin
     * wheel_motor_back_right_reset_Pin wheel_motor_back_right_direction_Pin */
    GPIO_InitStruct.Pin = wheel_motor_back_right_brake_Pin |
                          wheel_motor_back_right_reset_Pin |
                          wheel_motor_back_right_direction_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : wheel_motor_back_left_brake_Pin wheel_motor_back_left_esf_Pin
       wheel_motor_front_left_esf_Pin wheel_motor_back_left_reset_Pin
                             wheel_motor_back_left_coast_Pin LD3_Pin
       wheel_motor_back_left_mode_Pin wheel_motor_front_left_reset_Pin
                             wheel_motor_front_left_coast_Pin
       wheel_motor_front_left_mode_Pin LD2_Pin wheel_motor_front_left_direction_Pin
                             wheel_motor_front_left_brake_Pin */
    GPIO_InitStruct.Pin =
        wheel_motor_back_left_brake_Pin | wheel_motor_back_left_esf_Pin |
        wheel_motor_front_left_esf_Pin | wheel_motor_back_left_reset_Pin |
        wheel_motor_back_left_coast_Pin | LD3_Pin | wheel_motor_back_left_mode_Pin |
        wheel_motor_front_left_reset_Pin | wheel_motor_front_left_coast_Pin |
        wheel_motor_front_left_mode_Pin | LD2_Pin | wheel_motor_front_left_direction_Pin |
        wheel_motor_front_left_brake_Pin;
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

    /*Configure GPIO pin : wheel_motor_back_right_mode_Pin */
    GPIO_InitStruct.Pin   = wheel_motor_back_right_mode_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(wheel_motor_back_right_mode_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_io_proto_multicast_startNetworkingTask */
/**
 * @brief  Function implementing the NetStartTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_io_proto_multicast_startNetworkingTask */
__weak void io_proto_multicast_startNetworkingTask(void *argument)
{
    /* init code for LWIP */
    MX_LWIP_Init();
    /* USER CODE BEGIN 5 */
    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
    /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_test_msg_update */
/**
 * @brief Function implementing the testMsgUpdate thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_test_msg_update */
void test_msg_update(void *argument)
{
    /* USER CODE BEGIN test_msg_update */

    // TODO https://github.com/UBC-Thunderbots/Software/issues/1519
    // This is a placeholder task to test sending robot status NOT
    // associated with a ticket because how the robot status msgs will be
    // updated and sent is TBD
    ProtoMulticastCommunicationProfile_t *comm_profile =
        (ProtoMulticastCommunicationProfile_t *)argument;

    /* Infinite loop */
    for (;;)
    {
        io_proto_multicast_communication_profile_acquireLock(comm_profile);
        // TODO enable SNTP sys_now is currently only time since reset
        // https://github.com/UBC-Thunderbots/Software/issues/1518
        tbots_robot_msg.time_sent.epoch_timestamp_seconds = sys_now();
        io_proto_multicast_communication_profile_releaseLock(comm_profile);
        io_proto_multicast_communication_profile_notifyEvents(comm_profile,
                                                              PROTO_UPDATED);
        // run loop at 100hz
        osDelay(1 / 100 * MILLISECONDS_PER_SECOND);
    }
    /* USER CODE END test_msg_update */
}

/* MPU Configuration */

void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    /* Disables the MPU */
    HAL_MPU_Disable();
    /** Initializes and configures the Region and the memory to be protected
     */
    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress      = 0x30040000;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_256B;
    MPU_InitStruct.SubRegionDisable = 0x0;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    /** Initializes and configures the Region and the memory to be protected
     */
    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER1;
    MPU_InitStruct.BaseAddress      = 0x30044000;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_16KB;
    MPU_InitStruct.SubRegionDisable = 0x0;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    /** Initializes and configures the Region and the memory to be protected
     */
    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER2;
    MPU_InitStruct.BaseAddress      = 0x24000000;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_512KB;
    MPU_InitStruct.SubRegionDisable = 0x0;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL1;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    /* Enables the MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

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
