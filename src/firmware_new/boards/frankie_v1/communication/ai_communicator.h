#pragma once

#include "firmware_new/proto/control_fw.pb.h"

/**
 * Global Protubufs
 *
 * The following probufs are updated by the packets received in the multicast
 * group this robot is currently in. If the multicast group is not exchanging
 * packets with the specified protobuf as the payload, the receive_msg will not
 * be updated.
 *
 */
typedef struct AICommuncicator AICommuncicator_t;

/*
 * Starts an RTOS task to communicate with AI using
 * mutlicast groups.
 *
 * The multicast group provided will be joined, if the
 * group or port have been changed, power cycle the robot
 * to take effect.
 *
 * @param void
 * @returns nothing
 */
void ai_communicator_init(const char* multicast_group_ip, int port);

/*
 * Send a protobuf message
 *
 * @param msg_to_send A const reference to the poplated proto serialize and send
 */
void ai_communicator_send_proto(const SEND_MSG_TYPE& msg_to_send);

/*
 * Receive a protobuf message, blocks until msg is received.
 *
 * @returns control_msg A decoded proto received in the multicast group
 */
control_msg ai_communicator_recieve_proto();

/**
 * The following probufs are updated by the packets received in the multicast
 * group this robot is currently in. If the multicast group is not exchanging
 * packets with the specified protobuf as the payload, the receive_msg will not
 * be updated.
 */
struct AICommunicator                               
{                                                       
        robot_ack receive_msg;                      
        control_msg send_msg;                      
                                                         
        const int receive_msg_size;                     
        const int send_msg_size;                        
                                                        
        const pb_field_t receive_fields;                
        const pb_field_t send_fields;                   

        uint8_t buffer[receive_msg_size];
};                                                  
