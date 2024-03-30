#pragma once

#include <boost/asio/io_service.hpp>

#include "shared/constants.h"
#include "software/networking/udp_listener.h"
#include "software/networking/udp_sender.h"

class LatencyTesterNode
{
   public:
    LatencyTesterNode(const std::string& interface, const int listen_channel, const unsigned short listen_port,
                      const int send_channel, const unsigned short send_port,
                      ReceiveCallback receive_callback);

    ~LatencyTesterNode();

    void sendString(const std::string& message);

    virtual void onReceive(const char* message, const size_t&) = 0;

   private:
    boost::asio::io_service io_listener_service_;
    std::unique_ptr<UdpListener> listener_;

    boost::asio::io_service io_sender_service_;
    std::unique_ptr<UdpSender> sender_;

    // The thread running the io_service in the background. This thread will run for the
    // entire lifetime of the class
    std::thread listener_thread_;
    std::thread sender_thread_;
};
