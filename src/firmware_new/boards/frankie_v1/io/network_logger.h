#pragma once

/**
 * Logger Task
 *
 * Consumes msgs on the log queue, and sends it over the network with
 * the provided communication profile.
 *
 * @param communication_profile The profile to send the logs over
 */
void io_network_logger_task(void* network_profile);

/**
 * Initialize network logger with the queue id of the log msg queue
 *
 * @param log_message_queue_id The queue id of the log msg queue
 */
void io_network_logger_init(osMessageQueueId_t log_message_queue_id);
