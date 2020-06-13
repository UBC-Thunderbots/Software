#pragma once
/**
 * This file is an abstraction around LwIP to communicate with AI
 */

#include "shared/proto/tbots_robot_msg.pb.h"
#include "shared/proto/tbots_software_msgs.pb.h"

typedef struct AICommunicator AICommunicator_t;
typedef void (*vision_callback_t)(VisionMsg vision);
typedef void (*primitive_callback_t)(PrimitiveMsg primitive);

/**
 * Create an AICommunicator: contains all the networking required to communicate with AI
 *
 * @param multicast_address The multicast address or multicast "channel" the robot is
 * connected to
 * @param vision_port The port vision messages will be received
 * @param primitive_port The port primitive messages will be received
 * @param robot_status_port The port robot status messages should be sent to
 *
 * @return An ai_communicator with all the connections established
 */
AICommunicator_t* io_ai_communicator_create(const char* multicast_address,
                                            unsigned vision_port, unsigned primtive_port,
                                            unsigned robot_status_port,
                                            vision_callback_t vision_callback,
                                            primitive_callback_t primitive_callback);

/**
 * Destroy the given io_ai_communicator
 * @param io_ai_communicator The communicator
 */
void io_ai_communicator_destroy(AICommunicator_t* io_ai_communicator);

/**
 * Handles all the networking componenets needed to communicate with AI
 *
 */
void io_ai_communicator_networkingTaskHandler(void* io_ai_communicator);

/**
 * Send a TbotsRobotMsg to AI
 *
 * @param io_ai_communicator
 * @param robot_msg The protobuf message containing the RobotStatus
 */
void io_ai_communicator_sendTbotsRobotMsg(AICommunicator_t* io_ai_communicator,
                                          const TbotsRobotMsg robot_msg);

void io_ai_communicator_receiveVisionMsg(AICommunicator_t* io_ai_communicator,
                                         const VisionMsg vision_msg);

void io_ai_communicator_receivePrimitiveMsg(AICommunicator_t* io_ai_communicator,
                                            const PrimitiveMsg vision_msg);
