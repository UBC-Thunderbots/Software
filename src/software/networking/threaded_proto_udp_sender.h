#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>

#include "software/networking/proto_udp_sender.h"

template <class SendProto>
class ThreadedProtoUdpSender
{
   public:
    /**
     * Creates a ProtoUdpSender that sends the SendProto over the network on the
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
                           bool multicast);

    ~ThreadedProtoUdpSender();

    /**
     * Sends a protobuf message to the initialized ip address and port
     * This function returns after the message has been sent.
     *
     * @param message The protobuf message to send
     */
    void sendProto(const SendProto& message);

   private:
    // The io_service that will be used to service all network requests
    boost::asio::io_service io_service;
    ProtoUdpSender<SendProto> udp_sender;
    // The thread running the io_service in the background. This thread will run for the
    // entire lifetime of the class
    std::thread io_service_thread;
};

#include "software/networking/threaded_proto_udp_sender.tpp"
