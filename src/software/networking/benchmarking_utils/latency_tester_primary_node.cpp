#include "software/netowrking/benchmarking_utils/latency_tester_primary_node.h"

LatencyTesterPrimaryNode::LatencyTesterPrimaryNode(const int listen_channel, const unsigned short listen_port,
        const int send_channel, const unsigned short send_port, const int message_size, const std::chrono::milliseconds timeout_duration)
    : LatencyTesterNode(listen_channel, listen_port, send_channel, send_port),
      response_received_(false),
      num_timeouts_(0),
      timeout_duration_(timeout_duration)
{
    std::string charset = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890";
    send_buffer_.resize(message_size);
    for (int i = 0; i < message_size; i++)
    {
        send_buffer_[i] = charset[rand() % charset.size()];
    }
};

void LatencyTesterPrimaryNode::printStatistics()
{
    double sum = std::accumulate(latencies_.begin(), latencies_.end(), 0.0);
    double mean = sum / latencies_.size();

    double sq_sum = std::inner_product(latencies_.begin(), latencies_.end(), latencies_.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / latencies_.size() - mean * mean);

    LOG(INFO) << "Mean latency: " << mean << " ms";
    LOG(INFO) << "Standard deviation: " << stdev << " ms";
    LOG(INFO) << "Number of timeouts: " << num_timeouts_;
}

void LatencyTesterPrimaryNode::runTest(const int num_messages)
{
    LOG(INFO) << "Running test and sending " << num_messages << " messages of size" << send_buffer_.size() << " bytes.;

    for (int i = 0; i < num_messages; i++)
    {
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

    std::cv_status status;
    do 
    {
        last_send_time_ = std::chrono::system_clock::now();
        sendString(send_buffer_);
        {
            std::unique_lock<std::mutex> lock(response_mutex_);
            status = response_cv_.wait_for(response_mutex_, timeout_duration_, [this] { return response_received_; });
        }

        if (status == std::cv_status::timeout)
        {
            num_timeouts_++;
            LOG(INFO) << "Timeout, retrying";
        }
    } while (status == std::cv_status::timeout);
}

void LatencyTesterPrimaryNode::onReceive(const std::string& message)
{
    std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    std::chrono::milliseconds latency = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send_time_);
    latencies_.push_back(latency.count());

    std::lock_guard<std::mutex> lock(response_mutex_);
    response_received_ = true;
    response_cv_.notify_one();
}
