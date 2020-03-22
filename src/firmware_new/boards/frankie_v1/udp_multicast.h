#pragma once

#include "shared/proto/primitive_fw.pb.h"
typedef void (*primitive_callback_t)(PrimitiveMsg primitive);

/*
 * Join the multicast group to receive packets from AI. Starts a new thread
 * and handles packets asyncrhonously.
 *
 * @param multicast_address The IPV6 address to join
 * @param multicast_port The port to bind to listen for multicast packets from AI
 * @param send_port The port to bind to to send unicast packets to AI
 * @param callback The callback to run with the latest primitive protobuf
 *
 */

void udp_multicast_init(const char* multicast_address, unsigned multicast_port,
                        unsigned send_port, primitive_callback_t callback);

