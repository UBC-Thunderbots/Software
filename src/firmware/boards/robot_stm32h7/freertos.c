/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#pragma GCC diagnostic pop
#include "firmware/app/logger/logger.h"
#include "firmware/boards/robot_stm32h7/io/drivetrain.h"
#include "firmware/boards/robot_stm32h7/io/network_logger.h"
#include "firmware/boards/robot_stm32h7/io/power_monitor.h"
#include "firmware/boards/robot_stm32h7/io/proto_multicast.h"
#include "firmware/boards/robot_stm32h7/io/proto_multicast_communication_profile.h"
#include "firmware/boards/robot_stm32h7/io/robot_status.h"
#include "firmware/boards/robot_stm32h7/io/ublox_odinw262_communicator.h"
#include "firmware/boards/robot_stm32h7/tim.h"
#include "firmware/boards/robot_stm32h7/usart.h"
#include "shared/proto/robot_log_msg.nanopb.h"
#include "shared/proto/robot_status_msg.nanopb.h"
#include "shared/proto/tbots_software_msgs.nanopb.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */
// Networking
// the IPv6 multicast address, only ff02 is important, the rest is random
// see https://en.wikipedia.org/wiki/Solicited-node_multicast_address for why ff02 matters
#define MAX_MULTICAST_CHANNELS 16
#define MULTICAST_CHANNEL_LENGTH 21
const char MULTICAST_CHANNELS[MAX_MULTICAST_CHANNELS][MULTICAST_CHANNEL_LENGTH] = {
    "ff02::c3d0:42d2:bb01", "ff02::c3d0:42d2:bb02", "ff02::c3d0:42d2:bb03",
    "ff02::c3d0:42d2:bb04", "ff02::c3d0:42d2:bb05", "ff02::c3d0:42d2:bb06",
    "ff02::c3d0:42d2:bb07", "ff02::c3d0:42d2:bb08", "ff02::c3d0:42d2:bb09",
    "ff02::c3d0:42d2:bb10", "ff02::c3d0:42d2:bb11", "ff02::c3d0:42d2:bb12",
    "ff02::c3d0:42d2:bb13", "ff02::c3d0:42d2:bb14", "ff02::c3d0:42d2:bb15",
    "ff02::c3d0:42d2:bb16",
};

// the port robots are listening to for vision and primitives
const short unsigned int VISION_PORT    = 42069;
const short unsigned int PRIMITIVE_PORT = 42070;

// the port the AI receives msgs from the robot
const short unsigned int ROBOT_STATUS_PORT = 42071;
const short unsigned int ROBOT_LOGS_PORT   = 42072;

// the port to listen to for what side of the field to defend
const unsigned DEFENDING_SIDE_PORT = 42073;

// the timeout to recv a network packet
const int NETWORK_TIMEOUT_MS = 1000;

// maximum transfer unit of the network interface
// this is an int to avoid Wconversion with lwip
const short unsigned int MAXIMUM_TRANSFER_UNIT_BYTES = 1500;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

ProtoMulticastCommunicationProfile_t *robot_status_msg_sender_profile;
ProtoMulticastCommunicationProfile_t *robot_log_msg_sender_profile;
ProtoMulticastCommunicationProfile_t *vision_msg_listener_profile;
ProtoMulticastCommunicationProfile_t *primitive_msg_listener_profile;

static TbotsProto_Vision vision_msg;
static TbotsProto_RobotStatus robot_status_msg;
static TbotsProto_RobotLog robot_log_msg;
static TbotsProto_Primitive primitive_msg;

/* USER CODE END Variables */
/* Definitions for NetStartTask */
osThreadId_t NetStartTaskHandle;
uint32_t NetStartTaskBuffer[ 1024 ];
osStaticThreadDef_t NetStartTaskControlBlock;
const osThreadAttr_t NetStartTask_attributes = {
  .name = "NetStartTask",
  .cb_mem = &NetStartTaskControlBlock,
  .cb_size = sizeof(NetStartTaskControlBlock),
  .stack_mem = &NetStartTaskBuffer[0],
  .stack_size = sizeof(NetStartTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RobotStatusTask */
osThreadId_t RobotStatusTaskHandle;
uint32_t RobotStatusTaskBuffer[ 1024 ];
osStaticThreadDef_t RobotStatusTaskControlBlock;
const osThreadAttr_t RobotStatusTask_attributes = {
  .name = "RobotStatusTask",
  .cb_mem = &RobotStatusTaskControlBlock,
  .cb_size = sizeof(RobotStatusTaskControlBlock),
  .stack_mem = &RobotStatusTaskBuffer[0],
  .stack_size = sizeof(RobotStatusTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for VisionMsgTask */
osThreadId_t VisionMsgTaskHandle;
uint32_t VisionMsgTaskBuffer[ 1024 ];
osStaticThreadDef_t VisionMsgTaskControlBlock;
const osThreadAttr_t VisionMsgTask_attributes = {
  .name = "VisionMsgTask",
  .cb_mem = &VisionMsgTaskControlBlock,
  .cb_size = sizeof(VisionMsgTaskControlBlock),
  .stack_mem = &VisionMsgTaskBuffer[0],
  .stack_size = sizeof(VisionMsgTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PrimMsgTask */
osThreadId_t PrimMsgTaskHandle;
uint32_t PrimMsgTaskBuffer[ 1024 ];
osStaticThreadDef_t PrimMsgTaskControlBlock;
const osThreadAttr_t PrimMsgTask_attributes = {
  .name = "PrimMsgTask",
  .cb_mem = &PrimMsgTaskControlBlock,
  .cb_size = sizeof(PrimMsgTaskControlBlock),
  .stack_mem = &PrimMsgTaskBuffer[0],
  .stack_size = sizeof(PrimMsgTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RobotStatus */
osThreadId_t RobotStatusHandle;
uint32_t RobotStatusBuffer[ 1024 ];
osStaticThreadDef_t RobotStatusControlBlock;
const osThreadAttr_t RobotStatus_attributes = {
  .name = "RobotStatus",
  .cb_mem = &RobotStatusControlBlock,
  .cb_size = sizeof(RobotStatusControlBlock),
  .stack_mem = &RobotStatusBuffer[0],
  .stack_size = sizeof(RobotStatusBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RobotLogMsgSend */
osThreadId_t RobotLogMsgSendHandle;
uint32_t RobotLogMsgSendBuffer[ 1024 ];
osStaticThreadDef_t RobotLogMsgSendControlBlock;
const osThreadAttr_t RobotLogMsgSend_attributes = {
  .name = "RobotLogMsgSend",
  .cb_mem = &RobotLogMsgSendControlBlock,
  .cb_size = sizeof(RobotLogMsgSendControlBlock),
  .stack_mem = &RobotLogMsgSendBuffer[0],
  .stack_size = sizeof(RobotLogMsgSendBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for NetworkRobotLog */
osThreadId_t NetworkRobotLogHandle;
uint32_t NetworkRobotLogBuffer[ 1024 ];
osStaticThreadDef_t NetworkRobotLogControlBlock;
const osThreadAttr_t NetworkRobotLog_attributes = {
  .name = "NetworkRobotLog",
  .cb_mem = &NetworkRobotLogControlBlock,
  .cb_size = sizeof(NetworkRobotLogControlBlock),
  .stack_mem = &NetworkRobotLogBuffer[0],
  .stack_size = sizeof(NetworkRobotLogBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UbloxOdinTask */
osThreadId_t UbloxOdinTaskHandle;
uint32_t UbloxOdinTaskBuffer[ 1024 ];
osStaticThreadDef_t UbloxOdinTaskControlBlock;
const osThreadAttr_t UbloxOdinTask_attributes = {
  .name = "UbloxOdinTask",
  .cb_mem = &UbloxOdinTaskControlBlock,
  .cb_size = sizeof(UbloxOdinTaskControlBlock),
  .stack_mem = &UbloxOdinTaskBuffer[0],
  .stack_size = sizeof(UbloxOdinTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RobotStatusSend */
osThreadId_t RobotStatusSendHandle;
uint32_t RobotStatusSendBuffer[ 1024 ];
osStaticThreadDef_t RobotStatusSendControlBlock;
const osThreadAttr_t RobotStatusSend_attributes = {
  .name = "RobotStatusSend",
  .cb_mem = &RobotStatusSendControlBlock,
  .cb_size = sizeof(RobotStatusSendControlBlock),
  .stack_mem = &RobotStatusSendBuffer[0],
  .stack_size = sizeof(RobotStatusSendBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RobotLogProtoQ */
osMessageQueueId_t RobotLogProtoQHandle;
const osMessageQueueAttr_t RobotLogProtoQ_attributes = {
  .name = "RobotLogProtoQ"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void io_proto_multicast_startNetworkingTask(void *argument);
extern void io_proto_multicast_senderTask(void *argument);
extern void io_proto_multicast_listenerTask(void *argument);
extern void io_robot_status_task(void *argument);
extern void io_network_logger_task(void *argument);
extern void io_ublox_odinw262_communicator_task(void *argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of RobotLogProtoQ */
  RobotLogProtoQHandle = osMessageQueueNew (16, sizeof(TbotsProto_RobotLog), &RobotLogProtoQ_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of NetStartTask */
  NetStartTaskHandle = osThreadNew(io_proto_multicast_startNetworkingTask, NULL, &NetStartTask_attributes);

  /* creation of RobotStatusTask */
  RobotStatusTaskHandle = osThreadNew(io_proto_multicast_senderTask, (void *)robot_status_msg_sender_profile, &RobotStatusTask_attributes);

  /* creation of VisionMsgTask */
  VisionMsgTaskHandle = osThreadNew(io_proto_multicast_listenerTask, (void *)vision_msg_listener_profile, &VisionMsgTask_attributes);

  /* creation of PrimMsgTask */
  PrimMsgTaskHandle = osThreadNew(io_proto_multicast_listenerTask, (void *)primitive_msg_listener_profile, &PrimMsgTask_attributes);

  /* creation of RobotStatus */
  RobotStatusHandle = osThreadNew(io_robot_status_task, (void*)robot_status_msg_sender_profile, &RobotStatus_attributes);

  /* creation of RobotLogMsgSend */
  RobotLogMsgSendHandle = osThreadNew(io_proto_multicast_senderTask, (void *)robot_log_msg_sender_profile, &RobotLogMsgSend_attributes);

  /* creation of NetworkRobotLog */
  NetworkRobotLogHandle = osThreadNew(io_network_logger_task, (void*)robot_log_msg_sender_profile, &NetworkRobotLog_attributes);

  /* creation of UbloxOdinTask */
  UbloxOdinTaskHandle = osThreadNew(io_ublox_odinw262_communicator_task, NULL, &UbloxOdinTask_attributes);

  /* creation of RobotStatusSend */
  RobotStatusSendHandle = osThreadNew(io_proto_multicast_senderTask, (void*)robot_status_msg_sender_profile, &RobotStatusSend_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

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
  /* USER CODE BEGIN io_proto_multicast_startNetworkingTask */
    /* Infinite loop */
    for (;;)
    {
        osDelay(10000);
    }
  /* USER CODE END io_proto_multicast_startNetworkingTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void initIoNetworking(void)
{
    // TODO channel and robot_id need to be hooked up to the dials on the robot, when
    // available https://github.com/UBC-Thunderbots/Software/issues/1517
    unsigned short int channel = 0;

    // Initialize multicast communication
    io_proto_multicast_init(NETWORK_TIMEOUT_MS);

    primitive_msg_listener_profile = io_proto_multicast_communication_profile_create(
        "primitive_msg_listener_profile", MULTICAST_CHANNELS[channel], PRIMITIVE_PORT,
        &primitive_msg, TbotsProto_Primitive_fields, MAXIMUM_TRANSFER_UNIT_BYTES);

    vision_msg_listener_profile = io_proto_multicast_communication_profile_create(
        "vision_msg_listener_profile", MULTICAST_CHANNELS[channel], VISION_PORT,
        &vision_msg, TbotsProto_Vision_fields, MAXIMUM_TRANSFER_UNIT_BYTES);

    robot_status_msg_sender_profile = io_proto_multicast_communication_profile_create(
        "robot_status_msg_sender", MULTICAST_CHANNELS[channel], ROBOT_STATUS_PORT,
        &robot_status_msg, TbotsProto_RobotStatus_fields, MAXIMUM_TRANSFER_UNIT_BYTES);

    robot_log_msg_sender_profile = io_proto_multicast_communication_profile_create(
        "robot_log_msg_sender", MULTICAST_CHANNELS[channel], ROBOT_LOGS_PORT,
        &robot_log_msg, TbotsProto_RobotLog_fields, MAXIMUM_TRANSFER_UNIT_BYTES);

    // TODO (#2064) mainboard rev 2.0 doens't have a reset pin for the u-blox chip
    // so we can't enable the communicator. Uncomment this for mainboard rev 2.1
    // which should fix this issue.
    //
    // GpioPin_t *ublox_reset_pin =
    //     io_gpio_pin_create(ublox_reset_GPIO_Port, ublox_reset_Pin, ACTIVE_LOW);
    //
    // io_ublox_odinw262_communicator_init(&huart8, ublox_reset_pin, 5);
    //
    // Initialize network logger
    io_network_logger_init(RobotLogProtoQHandle);
}

void initIoDrivetrain(void)
{
    // Initialize a motor driver with the given suffix, on the given
    // timer channel
    GpioPin_t *reset_pin =
        io_gpio_pin_create(MOTOR_B_RESET_GPIO_Port, MOTOR_B_RESET_Pin, ACTIVE_HIGH);

    GpioPin_t *mode_pin =
        io_gpio_pin_create(MOTOR_B_MODE_GPIO_Port, MOTOR_B_MODE_Pin, ACTIVE_HIGH);

    GpioPin_t *direction_pin =
        io_gpio_pin_create(MOTOR_B_DIR_GPIO_Port, MOTOR_B_DIR_Pin, ACTIVE_HIGH);

    PwmPin_t *pwm_pin = io_pwm_pin_create(&htim1, TIM_CHANNEL_4);

    AllegroA3931MotorDriver_t *motor_driver =
        io_allegro_a3931_motor_driver_create(pwm_pin, reset_pin, mode_pin, direction_pin);

    io_allegro_a3931_motor_setPwmPercentage(motor_driver, 0.8f);

    TLOG_DEBUG("Starting PWM timers");

    // TIMER 1
    PwmPin_t *tim1_pwm_pin1 = io_pwm_pin_create(&htim1, TIM_CHANNEL_1);
    PwmPin_t *tim1_pwm_pin2 = io_pwm_pin_create(&htim1, TIM_CHANNEL_2);
    PwmPin_t *tim1_pwm_pin3 = io_pwm_pin_create(&htim1, TIM_CHANNEL_3);
    PwmPin_t *tim1_pwm_pin4 = io_pwm_pin_create(&htim1, TIM_CHANNEL_4);

    // TIMER 8
    PwmPin_t *tim8_pwm_pin1 = io_pwm_pin_create(&htim8, TIM_CHANNEL_1);
    PwmPin_t *tim8_pwm_pin2 = io_pwm_pin_create(&htim8, TIM_CHANNEL_4);

    io_pwm_pin_setPwm(tim1_pwm_pin1, 0.5);
    io_pwm_pin_setPwm(tim1_pwm_pin2, 0.5);
    io_pwm_pin_setPwm(tim1_pwm_pin3, 0.5);
    io_pwm_pin_setPwm(tim1_pwm_pin4, 0.5);
    io_pwm_pin_setPwm(tim8_pwm_pin1, 0.5);
    io_pwm_pin_setPwm(tim8_pwm_pin2, 0.5);

    TLOG_DEBUG("DONZO");
}

void initIoPowerMonitor(void)
{
    io_power_monitor_init(I2C1, INA226_ADDRESS,
                          INA226_MODE_CONT_SHUNT_AND_BUS | INA226_VBUS_140uS |
                              INA226_VBUS_140uS | INA226_AVG_1024);
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
