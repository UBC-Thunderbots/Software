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
/* USER CODE BEGIN Variables */

ProtoMulticastCommunicationProfile_t *tbots_robot_msg_sender_profile;
ProtoMulticastCommunicationProfile_t *vision_msg_listener_profile;
ProtoMulticastCommunicationProfile_t *primitive_msg_listener_profile;

static VisionMsg vision_msg;
static TbotsRobotMsg tbots_robot_msg;
static PrimitiveMsg primitive_msg;

/* USER CODE END Variables */
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

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void io_proto_multicast_startNetworkingTask(void *argument);
extern void io_proto_multicast_sender_task(void *argument);
extern void io_proto_multicast_listener_task(void *argument);
void test_msg_update(void *argument);

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
    /* add threads, ... */
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
        tbots_robot_msg.time_sent.epoch_timestamp_seconds = sys_now();
        io_proto_multicast_communication_profile_releaseLock(comm_profile);
        io_proto_multicast_communication_profile_notifyEvents(comm_profile,
                                                              PROTO_UPDATED);
        // run loop at 100hz
        osDelay(1 / 100 * MILLISECONDS_PER_SECOND);
    }
    /* USER CODE END test_msg_update */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
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


/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
