#include "software/networking/benchmarking_utils/latency_tester_node.h"

LatencyTesterNode::LatencyTesterNode(const int listen_channel,
                                     const unsigned short listen_port,
                                     const int send_channel,
                                     const unsigned short send_port,
                                     ReceiveCallback receive_callback)
    : io_listener_service_(),
      listener_(io_listener_service_, ROBOT_MULTICAST_CHANNELS.at(listen_channel),
                listen_port, true, receive_callback),
      io_sender_service_(),
      sender_(io_sender_service_, ROBOT_MULTICAST_CHANNELS.at(send_channel), send_port,
              true)
{
}

void LatencyTesterNode::sendString(const std::string& message)
{
    sender_.sendString(message);
}
