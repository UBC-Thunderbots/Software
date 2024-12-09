#pragma once

#include "software/networking/benchmarking_utils/latency_tester_node.h"

/**
 * A node that receives messages and bounces back the message to the sender.
 *
 * The aim of this node is to bounce back messages as quickly as possible to provide the driving node with an estimate
 * of the latency between the two nodes. As such, this node doesn't perform any processing or logging.
 */
class LatencyTesterSecondaryNode : public LatencyTesterNode
{
   public:
    /**
     * Creates a multicast node that bounces back messages to the sender.
     *
     * @param interface The network interface to use for network communication
     * @param listen_channel The channel to listen on for incoming messages
     * @param listen_port The port to listen on for incoming messages
     * @param send_channel The channel to send messages on
     * @param send_port The port to send messages on
     */
    LatencyTesterSecondaryNode(const std::string& interface, const int listen_channel, const unsigned short listen_port,
                               const int send_channel, const unsigned short send_port);

    /**
     * Creates a unicast node that bounces back messages to the given sender.
     *
     * @param interface The network interface to use for network communication
     * @param listen_port The port to listen on for incoming messages
     * @param send_ip IP address of the sender to bounce messages back to
     * @param send_port The port to send messages on
     */
    LatencyTesterSecondaryNode(const std::string& interface, const unsigned short listen_port,
            const std::string& send_ip, const unsigned short send_port);

  private:
    /**
     * Callback function that bounces back the received message to the sender.
     *
     * @param message Contexts of the message received
     * @param size Size of the message received
     */
    void onReceive(const char* message, const size_t& size) override;
};
