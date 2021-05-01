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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"

#include "cmsis_os.h"
#include "main.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "firmware/app/logger/logger.h"
#include "firmware_new/boards/frankie_v1/io/drivetrain.h"
#include "firmware_new/boards/frankie_v1/io/network_logger.h"
#include "firmware_new/boards/frankie_v1/io/proto_multicast_communication_profile.h"
#include "firmware_new/boards/frankie_v1/io/proto_multicast_communication_tasks.h"
#include "firmware_new/boards/frankie_v1/io/ublox_odinw262_communicator.h"
#include "firmware_new/boards/frankie_v1/tim.h"
#include "firmware_new/boards/frankie_v1/usart.h"
#include "shared/constants.h"
#include "shared/proto/robot_log_msg.nanopb.h"
#include "shared/proto/robot_status_msg.nanopb.h"
#include "shared/proto/tbots_software_msgs.nanopb.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PDF */

/* USER CODE END PDF */

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
const osThreadAttr_t NetStartTask_attributes = {
    .name       = "NetStartTask",
    .priority   = (osPriority_t)osPriorityNormal,
    .stack_size = 1024 * 4};
/* Definitions for RobotStatusTask */
osThreadId_t RobotStatusTaskHandle;
const osThreadAttr_t RobotStatusTask_attributes = {
    .name       = "RobotStatusTask",
    .priority   = (osPriority_t)osPriorityNormal1,
    .stack_size = 1024 * 4};
/* Definitions for VisionMsgTask */
osThreadId_t VisionMsgTaskHandle;
const osThreadAttr_t VisionMsgTask_attributes = {
    .name       = "VisionMsgTask",
    .priority   = (osPriority_t)osPriorityNormal,
    .stack_size = 1024 * 4};
/* Definitions for PrimMsgTask */
osThreadId_t PrimMsgTaskHandle;
const osThreadAttr_t PrimMsgTask_attributes = {.name     = "PrimMsgTask",
                                               .priority = (osPriority_t)osPriorityNormal,
                                               .stack_size = 1024 * 4};
/* Definitions for testMsgUpdate */
osThreadId_t testMsgUpdateHandle;
const osThreadAttr_t testMsgUpdate_attributes = {
    .name       = "testMsgUpdate",
    .priority   = (osPriority_t)osPriorityNormal,
    .stack_size = 1024 * 4};
/* Definitions for RobotLogMsgSend */
osThreadId_t RobotLogMsgSendHandle;
const osThreadAttr_t RobotLogMsgSend_attributes = {
    .name       = "RobotLogMsgSend",
    .priority   = (osPriority_t)osPriorityNormal,
    .stack_size = 1024 * 4};
/* Definitions for NetworkRobotLog */
osThreadId_t NetworkRobotLogHandle;
const osThreadAttr_t NetworkRobotLog_attributes = {
    .name       = "NetworkRobotLog",
    .priority   = (osPriority_t)osPriorityNormal,
    .stack_size = 1024 * 4};
/* Definitions for UbloxOdinTask */
osThreadId_t UbloxOdinTaskHandle;
const osThreadAttr_t UbloxOdinTask_attributes = {
    .name       = "UbloxOdinTask",
    .priority   = (osPriority_t)osPriorityNormal,
    .stack_size = 1024 * 4};
/* Definitions for RobotLogProtoQ */
osMessageQueueId_t RobotLogProtoQHandle;
const osMessageQueueAttr_t RobotLogProtoQ_attributes = {.name = "RobotLogProtoQ"};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void io_proto_multicast_startNetworkingTask(void *argument);
extern void io_proto_multicast_sender_task(void *argument);
extern void io_proto_multicast_listener_task(void *argument);
void test_msg_update(void *argument);
extern void io_network_logger_task(void *argument);
extern void io_ublox_odinw262_communicator_task(void *argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
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
    RobotLogProtoQHandle =
        osMessageQueueNew(16, sizeof(TbotsProto_RobotLog), &RobotLogProtoQ_attributes);

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
                    (void *)robot_status_msg_sender_profile, &RobotStatusTask_attributes);

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
        osThreadNew(test_msg_update, (void *)robot_status_msg_sender_profile,
                    &testMsgUpdate_attributes);

    /* creation of RobotLogMsgSend */
    RobotLogMsgSendHandle =
        osThreadNew(io_proto_multicast_sender_task, (void *)robot_log_msg_sender_profile,
                    &RobotLogMsgSend_attributes);

    /* creation of NetworkRobotLog */
    NetworkRobotLogHandle =
        osThreadNew(io_network_logger_task, (void *)robot_log_msg_sender_profile,
                    &NetworkRobotLog_attributes);

    /* creation of UbloxOdinTask */
    UbloxOdinTaskHandle =
        osThreadNew(io_ublox_odinw262_communicator_task, NULL, &UbloxOdinTask_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* USER CODE END RTOS_THREADS */
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
        osDelay(1);
    }
    /* USER CODE END io_proto_multicast_startNetworkingTask */
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
        robot_status_msg.time_sent.epoch_timestamp_seconds = sys_now();

        // We change the power status values randomly so that robot diagnostics
        // can "see" this robot on the network. This is a stopgap until we have
        // actual values for RobotStatus
        robot_status_msg.power_status.battery_voltage   = (float)(sys_now() % 100);
        robot_status_msg.power_status.capacitor_voltage = (float)(sys_now() % 100);
        io_proto_multicast_communication_profile_releaseLock(comm_profile);
        io_proto_multicast_communication_profile_notifyEvents(comm_profile,
                                                              PROTO_UPDATED);
        // run loop at 100hz
        osDelay((unsigned int)MILLISECONDS_PER_SECOND / 10);
    }
    /* USER CODE END test_msg_update */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void initIoNetworking()
{
    // TODO channel and robot_id need to be hooked up to the dials on the robot, when
    // available https://github.com/UBC-Thunderbots/Software/issues/1517
    unsigned short int channel = 0;

    // initialize multicast communication
    io_proto_multicast_communication_init(NETWORK_TIMEOUT_MS);

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

    // initialize ublox
    GpioPin_t *ublox_reset_pin =
        io_gpio_pin_create(ublox_reset_GPIO_Port, ublox_reset_Pin, ACTIVE_LOW);

    io_ublox_odinw262_communicator_init(&huart8, ublox_reset_pin, 5);

    // initialize network logger
    io_network_logger_init(RobotLogProtoQHandle);
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

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
