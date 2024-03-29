#pragma once

#include "software/networking/benchmarking_utils/latency_tester_node.h"

class LatencyTesterSecondaryNode : public LatencyTesterNode
{
    public:
        LatencyTesterSecondaryNode(const int listen_channel, const unsigned short listen_port,
                const int send_channel, const unsigned short send_port);

        virtual void onReceive(const char* message, const size_t& size) override;
};
