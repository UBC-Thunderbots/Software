#pragma once
#include "pb.h"

typedef struct ProtoMulticastListenerProfile ProtoMulticastListenerProfile_t;
typedef struct ProtoMulticastSenderProfile ProtoMulticastSenderProfile_t;

// TODO docs
void io_proto_multicast_sender_Task(void* arg);
void io_proto_multicast_listener_Task(void* arg);

/**
 * Create an ProtoMulticastSender/ListenerProfile
 *
 * contains all the networking and protobuf information required to join a multicast group
 * and receive/send proto
 *
 * @param multicast_address [in] The multicast channel the robot should join
 * @param port [in] The port to bind to
 * @param message_fields [in] The nanopb message fields used to seriallize/deserialize the
 * msg example: if the msg type is TestMsg, the fields are defined as TestMsg_fields
 * @param message_max_size [in] The maximum known size of the protobuf. If the maximum
 *        size is undeterminable (i.e repated fields), this value is recommended to be
 *        set at MAXIMUM_TRANSFER_UNIT (MTU) of the network packet.
 *
 * NOTE: We use a void pointer for protobuf_struct to make the sender/listener task
 * interface generic to any protobuf message type. nanopb uses message_fields to
 * understand the contents of the given struct/message.
 *
 * sender_profile_create specific:
 *
 * @param sending_rate_hertz [in] How frequently should the protobuf_struct be
 * deserialized and sent
 * @param protobuf_struct [out] This struct to serialized and send over the network.
 *
 * listener_profile_create specific:
 *
 * @param timeout_milliseconds [in] How long to wait before triggerent a timeout.
 * @param timeout_callback [in] The calllback to trigger when the timeout occurs.
 * @param protobuf_struct [out] The serialized proto in the incoming network packet, will
 * get deserialized into this struct.
 *
 * @return ProtoMulticastSender/ListenerProfile_t the profile to provide to the task
 *
 */
ProtoMulticastSenderProfile_t* io_proto_multicast_sender_profile_create(
    const char* multicast_address, uint16_t port, const void* protobuf_struct,
    const pb_field_t* message_fields, uint16_t message_max_size,
    uint16_t sending_rate_hertz);

ProtoMulticastListenerProfile_t* io_proto_multicast_listener_profile_create(
    const char* multicast_address, uint16_t port, const void* protobuf_struct,
    const pb_field_t* message_fields, uint16_t message_max_size,
    uint16_t timeout_milliseconds, void (*timeout_callback)(void));

/**
 * Destroy the given ProtoMulticastListenerProfile_t
 *
 * @param listener_profile The profile to delete
 */
void io_proto_multicast_listener_profile_destroy(
    ProtoMulticastListenerProfile_t* listener_profile);

/**
 * Destroy the given ProtoMulticastSenderProfile_t
 *
 * @param sender_profile The profile to delete
 */
void io_proto_multicast_sender_profile_destroy(
    ProtoMulticastSenderProfile_t* sender_profile);
