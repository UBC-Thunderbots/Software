#include "software/networking/benchmarking/latency_tester_node.h"

LatencyTesterNode::LatencyTesterNode(const int listen_channel, const unsigned short listen_port,
                                     const int send_channel, const unsigned short send_port)
    : io_listener_service_(),
      listener_(io_listener_service_, ROBOT_MULTICAST_CHANNELS.at(listen_channel), listen_port),
      io_sender_service_(),
      sender_(io_sender_service_, ROBOT_MULTICAST_CHANNELS.at(send_channel), send_port)
{
}

void LatencyTesterNode::sendString(const std::string& message)
{
    sender_.sendString(message);
}
