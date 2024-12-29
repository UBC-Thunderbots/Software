#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <optional>
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
     * Any callers must check the error string to see if the initialization was successful
     * before using the object.
     *
     * @param ip_address The ip address to send data on
     * (IPv4 in dotted decimal or IPv6 in hex string)
     *  example IPv4: 192.168.0.2
     *  example IPv6: ff02::c3d0:42d2:bb8
     * @param port The port to send SendProto data on
     * @param interface The interface to send data on
     * @param multicast If true, joins the multicast group of given ip_address
     * @param error An optional user-provided string that will be set to an error message
     * if an error occurs
     */
    ThreadedProtoUdpSender(const std::string& ip_address, unsigned short port,
                           const std::string& interface, bool multicast,
                           std::optional<std::string>& error)
        : ThreadedUdpSender(ip_address, port, interface, multicast, error)
    {
    }

    /**
     * Get the interface that this sender is sending on.
     *
     * @return The interface as a string
     */
    std::string getInterface() const;

    /**
     * Get the IP address that this sender is sending to.
     *
     * @return The IP address as a string
     */
    std::string getIpAddress() const;

    /**
     * Sends a protobuf message to the initialized ip address and port
     * This function returns after the message has been sent.
     *
     * @param message The protobuf message to send
     * @param async If true, the message will be sent asynchronously otherwise it will be
     * send synchronously
     */
    void sendProto(const SendProto& message, bool async = false);


   private:
    std::string data_buffer;
};

template <class SendProto>
std::string ThreadedProtoUdpSender<SendProto>::getInterface() const
{
    return ThreadedUdpSender::getInterface();
}

template <class SendProto>
std::string ThreadedProtoUdpSender<SendProto>::getIpAddress() const
{
    return ThreadedUdpSender::getIpAddress();
}

template <class SendProto>
void ThreadedProtoUdpSender<SendProto>::sendProto(const SendProto& message, bool async)
{
    message.SerializeToString(&data_buffer);
    sendString(data_buffer, async);
}
