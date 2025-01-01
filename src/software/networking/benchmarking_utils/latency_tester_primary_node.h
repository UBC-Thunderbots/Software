#pragma once

#include "software/networking/benchmarking_utils/latency_tester_node.h"

/**
 * Driver node for the network benchmarking test.
 */
class LatencyTesterPrimaryNode : public LatencyTesterNode
{
   public:
    /**
     * Sets up a multicast node to drive the network benchmarking test.
     *
     * @param interface The network interface to use for network communication
     * @param listen_channel The channel to listen on for responses
     * @param listen_port The port to listen on for responses
     * @param send_channel The channel to send requests on
     * @param send_port The port to send requests on
     * @param message_size The size of the messages to send
     * @param timeout_duration The duration to wait for a response before timing out
     */
    LatencyTesterPrimaryNode(const std::string& interface, const int listen_channel,
                             const unsigned short listen_port, const int send_channel,
                             const unsigned short send_port, const int message_size,
                             const std::chrono::milliseconds& timeout_duration);

    /**
     * Sets up a unicast node to drive the network benchmarking test.
     *
     * @param interface The network interface to use for network communication
     * @param listen_port The port to listen on for responses
     * @param send_ip IP address of the node to bounce messages off of to test round trip
     * times
     * @param send_port The port to send requests on
     * @param message_size The size of the messages to send
     * @param timeout_duration The duration to wait for a response before timing out
     */
    LatencyTesterPrimaryNode(const std::string& interface,
                             const unsigned short listen_port, const std::string& send_ip,
                             const unsigned short send_port, const int message_size,
                             const std::chrono::milliseconds& timeout_duration);

    /**
     * Logs round trip time statistics for the test.
     */
    void printStatistics();

    /**
     * Runs the network benchmarking test.
     *
     * @param num_messages The number of messages to send
     */
    void runTest(const int num_messages);

   private:
    /**
     * Records the round trip by noting the time the response was received from the
     * request driven during `runTest`.
     *
     * @param message Contents of the message received
     * @param size Size of the message received
     */
    void onReceive(const char* message, const size_t& size) override;

    /**
     * Initializes the node to start a new benchmarking test and generates a random
     * message to send for the given size.
     *
     * The same message is sent for all requests to eliminate the effect of message
     * generation on the round trip time.
     *
     * @param message_size The size of the messages to send
     * @param timeout_duration The duration to wait for a response before timing out
     */
    void initialize(const int message_size,
                    const std::chrono::milliseconds& timeout_duration);

    /**
     * Sends a transaction to the network.
     */
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
