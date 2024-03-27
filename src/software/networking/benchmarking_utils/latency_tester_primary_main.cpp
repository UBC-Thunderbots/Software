#include "software/networking/benchmarking_utils/latency_tester_node.h"

class LatencyTesterPrimaryNode : public LatencyTesterNode
{
    public:
        LatencyTesterPrimaryNode(const int listen_channel, const unsigned short listen_port,
                const int send_channel, const unsigned short send_port)
            : LatencyTesterNode(listen_channel, listen_port, send_channel, send_port) {}

        inline void onReceive(const std::string& message) override
        {
            h
        }
};
