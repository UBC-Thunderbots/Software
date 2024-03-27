#pragma once

#include "shared/constants.h"
#include "software/networking/udp_listener.h"
#include "software/networking/udp_sender.h"

#include <boost/asio/io_service.hpp>

class LatencyTesterNode
{
    public:
        LatencyTesterNode(const int listen_channel, const unsigned short listen_port,
                const int send_channel, const unsigned short send_port);

        void sendString(const std::string& message);

        virtual void onReceive(const std::string& message) = 0;

    private:
        boost::asio::io::service io_listener_service_;
        UdpListener listener_;

        boost::asio::io::service io_sender_service_;
        UdpSender sender_;
};
