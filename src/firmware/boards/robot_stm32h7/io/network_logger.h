#pragma once
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "proto/robot_log_msg.nanopb.h"

/**
 * Logger Task
 *
 * Sends logs over the given communication profile
 *
 * @param communication_profile The profile to send the logs over
 */
void io_network_logger_task(void* communication_profile);

/**
 * Initialize network logger with the queue id of the robot log msg queue
 *
 * @param log_message_queue_id The queue id of the robot log msg queue
 */
void io_network_logger_init(osMessageQueueId_t log_message_queue_id);

/**
 * Queues up the RobotLog proto to be sent by the io_network_logger_task
 *
 * @param robot_log The log msg to send
 */
void io_network_logger_handleRobotLog(TbotsProto_RobotLog robot_log);
