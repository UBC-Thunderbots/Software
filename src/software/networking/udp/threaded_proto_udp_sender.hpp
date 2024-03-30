#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>

#include "software/networking/udp/threaded_udp_sender.h"

template <class SendProto>
class ThreadedProtoUdpSender : private ThreadedUdpSender
{
   public:
    /**
     * Creates a UdpSender that sends the SendProto over the network on the
     * given address and port.
     *
     * @param ip_address The ip address to send data on
     * (IPv4 in dotted decimal or IPv6 in hex string)
     *  example IPv4: 192.168.0.2
     *  example IPv6: ff02::c3d0:42d2:bb8%wlp4s0 (the interface is specified after %)
     * @param port The port to send SendProto data on
     * @param multicast If true, joins the multicast group of given ip_address
     */
    ThreadedProtoUdpSender(const std::string& ip_address, unsigned short port,
                           bool multicast)
        : ThreadedUdpSender(ip_address, port, multicast)
    {
    }

    /**
     * Sends a protobuf message to the initialized ip address and port
     * This function returns after the message has been sent.
     *
     * @param message The protobuf message to send
     */
    void sendProto(const SendProto& message);

   private:
    std::string data_buffer;
};

template <class SendProto>
void ThreadedProtoUdpSender<SendProto>::sendProto(const SendProto& message)
{
    message.SerializeToString(&data_buffer);
    sendString(data_buffer);
}
