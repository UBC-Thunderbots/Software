#include "software/networking/benchmarking_utils/latency_tester_primary_node.h"

#include "software/logger/logger.h"
#include "software/math/statistics_functions.hpp"


LatencyTesterPrimaryNode::LatencyTesterPrimaryNode(
    const std::string& interface, const int listen_channel, const unsigned short listen_port, const int send_channel,
    const unsigned short send_port, const int message_size,
    const std::chrono::milliseconds& timeout_duration)
    : LatencyTesterNode(interface, listen_channel, listen_port, send_channel, send_port,
                        std::bind(&LatencyTesterPrimaryNode::onReceive, this,
                                  std::placeholders::_1, std::placeholders::_2)),
      response_received_(false),
      num_timeouts_(0),
      timeout_duration_(timeout_duration)
{
    std::string charset =
        "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890";
    send_buffer_.resize(message_size);
    for (int i = 0; i < message_size; i++)
    {
        send_buffer_[i] = charset[rand() % charset.size()];
    }
};

void LatencyTesterPrimaryNode::printStatistics()
{
    double latency_mean  = mean<long int>(latencies_);
    double latency_stdev = stdevSample<long int>(latencies_);

    LOG(INFO) << "Number of messages sent: " << latencies_.size();
    LOG(INFO) << "Mean latency: " << latency_mean << " ms";
    LOG(INFO) << "Standard deviation: " << latency_stdev << " ms";
    LOG(INFO) << "Number of timeouts: " << num_timeouts_;
}

void LatencyTesterPrimaryNode::runTest(const int num_messages)
{
    LOG(INFO) << "Running test and sending " << num_messages << " messages of size "
              << send_buffer_.size() << " bytes.";

    for (int i = 0; i < num_messages; i++)
    {
        LOG(INFO) << "Sending message " << i << " of " << num_messages;
        sendTransaction();
    }

    printStatistics();
}

void LatencyTesterPrimaryNode::sendTransaction()
{
    {
        std::lock_guard<std::mutex> lock(response_mutex_);
        response_received_ = false;
    }

    bool status = false;
    do
    {
        last_send_time_ = std::chrono::system_clock::now();
        sendString(send_buffer_);
        {
            std::unique_lock<std::mutex> lock(response_mutex_);
            status = response_cv_.wait_for(lock, timeout_duration_,
                                           [this] { return response_received_; });
        }

        if (!status)
        {
            num_timeouts_++;
            LOG(INFO) << "Timeout, retrying";
        }
    } while (!status);
}

void LatencyTesterPrimaryNode::onReceive(const char*, const size_t& size)
{
    std::chrono::time_point<std::chrono::system_clock> now =
        std::chrono::system_clock::now();
    std::chrono::milliseconds latency =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send_time_);
    latencies_.push_back(latency.count());

    LOG(INFO) << "Received response with size: " << size << " bytes";
    std::lock_guard<std::mutex> lock(response_mutex_);
    response_received_ = true;
    response_cv_.notify_one();
}
