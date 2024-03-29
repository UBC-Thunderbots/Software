#pragma once

#include <boost/asio/io_service.hpp>

#include "shared/constants.h"
#include "software/networking/udp_listener.h"
#include "software/networking/udp_sender.h"

class LatencyTesterNode
{
   public:
    LatencyTesterNode(const int listen_channel, const unsigned short listen_port,
                      const int send_channel, const unsigned short send_port,
                      ReceiveCallback receive_callback);

    void sendString(const std::string& message);

    virtual void onReceive(const char* message, const size_t&) = 0;

   private:
    boost::asio::io_service io_listener_service_;
    UdpListener listener_;

    boost::asio::io_service io_sender_service_;
    UdpSender sender_;
};
