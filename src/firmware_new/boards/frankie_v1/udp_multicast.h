#pragma once

/*
 * Join the multicast group to receive packets from AI. Starts a new thread
 * and handles packets asyncrhonously.
 *
 * @param multicast_address The IPV6 address to join
 * @param multicast_port The port to bind to listen for multicast packets from AI
 * @param send_port The port to bind to to send unicast packets to AI
 *
 */
void udp_multicast_init(const char* multicast_address, unsigned multicast_port,
                        unsigned send_port);
