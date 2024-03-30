#pragma once

#include "software/networking/benchmarking_utils/latency_tester_node.h"

class LatencyTesterPrimaryNode : public LatencyTesterNode
{
   public:
    LatencyTesterPrimaryNode(const std::string& interface, const int listen_channel, const unsigned short listen_port,
                             const int send_channel, const unsigned short send_port,
                             const int message_size,
                             const std::chrono::milliseconds& timeout_duration);

    void printStatistics();

    void runTest(const int num_messages);

    virtual void onReceive(const char* message, const size_t& size) override;

   private:
    void sendTransaction();

    std::string send_buffer_;

    std::mutex response_mutex_;
    std::condition_variable response_cv_;
    bool response_received_;

    int num_timeouts_;

    std::chrono::milliseconds timeout_duration_;
    std::chrono::time_point<std::chrono::system_clock> last_send_time_;
    std::vector<long int> latencies_;
};
