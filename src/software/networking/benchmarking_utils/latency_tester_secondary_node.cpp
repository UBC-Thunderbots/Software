#include "software/networking/benchmarking_utils/latency_tester_secondary_node.h"

LatencyTesterSecondaryNode::LatencyTesterSecondaryNode(const std::string& interface, const int listen_channel,
                                                       const unsigned short listen_port,
                                                       const int send_channel,
                                                       const unsigned short send_port)
    : LatencyTesterNode(interface, listen_channel, listen_port, send_channel, send_port,
                        std::bind(&LatencyTesterSecondaryNode::onReceive, this,
                                  std::placeholders::_1, std::placeholders::_2))
{
}

void LatencyTesterSecondaryNode::onReceive(const char* message, const size_t& size)
{
    sendString(std::string(message));
}
