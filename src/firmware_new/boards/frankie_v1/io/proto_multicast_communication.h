#pragma once

/**
 * This file is an abstraction around LwIP to communicate with AI
 */

#include "shared/proto/tbots_robot_msg.pb.h"
#include "shared/proto/tbots_software_msgs.pb.h"

typedef struct ProtoMulticastListenerProfile ProtoMulticastCommunicationProfile_t;
typedef struct ProtoMulticastSenderProfile ProtoMulticastCommunicationProfile_t;

/**
 * Create an ProtoMulticastListenerProfile
 *
 * contains all the networking and protobuf information required send/receive proto
 * over a multicast group
 *
 * @param multicast_address [in] The multicast address/"channel" the robot should connect to
 *
 * @param port [in] The port to bind to
 *
 * @param protobuf_struct [out] A pointer to the protobuf struct. We don't care aboout the type
 *        of the protobuf struct (hence the void pointer). The serialized proto in the incoming
 *        network packet, will get deserialized into this struct.
 *
 * @param message_fields [in] The nanopb message fields used to seriallize/deserialize msg
 *        if the msg type is TestMsg, the fields are defined as TestMsg_fields
 *
 * @param message_max_size [in] The maximum known size of the protobuf. If the maximum size
 *        is undeterminable (i.e repated fields), this value is recommended to be set at
 *        MAXIMUM_TRANSFER_UNIT (MTU) of the network packet
 *
 * @param timeout_milliseconds [in] How long to wait before triggerent a timeout.
 *
 * @return A ProtoMulticastConnectionProfile_t which can be given to a proto_multicast_listener_Task
 */
ProtoMulticastListenerProfile_t* io_proto_multicast_listener_profile_create(
    const char* multicast_address, uint16_t port, const void* protobuf_struct,
    const pb_field_t[] message_fields, uint16_t message_max_size,
    uint16_t timeout_millisecoonds, void (*timeout_callback)(void));

/**
 * Create an ProtoMulticastCommunicationProfile
 *
 * contains all the networking and protobuf information required send/receive proto
 * over a multicast group
 *
 * @param multicast_address [in] The multicast address/"channel" the robot should connect to
 *
 * @param port [in] The port to bind to
 *
 * @param protobuf_struct [in] A pointer to the protobuf struct. We don't care aboout the type
 *        of the protobuf struct (hence the void pointer). This protobuf struct will get serialized
 *        and sent over the network.
 *
 * @param message_fields [in] The nanopb message fields used to seriallize/deserialize msg
 *        if the msg type is TestMsg, the fields are defined as TestMsg_fields
 *
 * @param message_max_size [in] The maximum known size of the protobuf. If the maximum size
 *        is undeterminable (i.e repated fields), this value is recommended to be set at
 *        MAXIMUM_TRANSFER_UNIT (MTU) of the network packet
 *
 * @param sending_rate_hertz [in] How frequently should the msg be deserialized and sent
 *
 * @return A ProtoMulticastConnectionProfile_t which can be given to a proto_multicast_listener_Task
 */
ProtoMulticastSenderProfile_t* io_proto_multicast_sender_profile_create(
    const char* multicast_address, uint16_t port, const void* protobuf_struct,
    const pb_field_t[] message_fields, uint16_t message_max_size,
    uint16_t sending_rate_hertz);

/**
 * Destroy the given ProtoMulticastListenerProfile
 *
 * @param connection_profile The profile to delete
 */
void io_proto_multicast_listener_profile_destroy(
    ProtoMulticastCommunicationProfile_t* connection_profile);

/**
 * Destroy the given ProtoMulticastSenderProfile
 *
 * @param connection_profile The profile to delete
 */
void io_proto_multicast_sender_profile_destroy(
    ProtoMulticastCommunicationProfile_t* connection_profile);

/**
 * TODO
 */
void io_proto_multicast_sender_Task(
    ProtoMulticastCommunicationProfile_t* communication_profile);

void io_proto_multicast_listener_Task(
    ProtoMulticastCommunicationProfile_t* communication_profile);
