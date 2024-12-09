#pragma once

#include <boost/asio/io_service.hpp>

#include "shared/constants.h"
#include "software/networking/udp/udp_listener.h"
#include "software/networking/udp/udp_sender.h"

/**
 * A node in a network benchmarking test.
 */
class LatencyTesterNode
{
   public:
    /**
     * Sets up a multicast benchmarking node.
     *
     * @param interface The interface to use for communication
     * @param listen_channel The multicast channel to listen on
     * @param listen_port The port to listen on
     * @param send_channel The multicast channel to send on
     * @param send_port The port to send on
     */
    LatencyTesterNode(const std::string& interface, const int listen_channel, const unsigned short listen_port,
                      const int send_channel, const unsigned short send_port);

    /**
     * Sets up a unicast benchmarking node.
     *
     * @param interface The interface to use for communication
     * @param listen_port The port to listen on
     * @param send_ip Unicast IP address of the endpoint to send to
     * @param send_port The port to send on
     */
    LatencyTesterNode(const std::string& interface, const unsigned short listen_port, const std::string& send_ip,
            const unsigned short send_port);

    /**
     * Cleans up the network resources
     */
    ~LatencyTesterNode();

    /**
     * Sends a message to the other network benchmarking node(s)
     *
     * @param message The message to send
     */
    void sendString(const std::string& message);

   private:
    /**
     * Callback function that is called when a message is received from another network benchmarking node.
     *
     * @param message Contents of the message received
     * @param size Size of the message received
     */
    virtual void onReceive(const char* message, const size_t& size) = 0;

    /**
     * Starts asynchronous threads for the networking resources.
     */
    void startServiceThreads();

    boost::asio::io_service io_listener_service_;
    std::unique_ptr<UdpListener> listener_;

    boost::asio::io_service io_sender_service_;
    std::unique_ptr<UdpSender> sender_;

    // The thread running the io_service in the background. This thread will run for the
    // entire lifetime of the class
    std::thread listener_thread_;
    std::thread sender_thread_;
};
