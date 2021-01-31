#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>

#include "software/networking/proto_multicast_sender.h"

template <class SendProto>
class ThreadedProtoMulticastSender
{
   public:
    /**
     * Creates a ProtoMulticastSender that sends the SendProto over the network on the
     * given address and port.
     *
     * @param ip_address The ip address of the multicast group to send data on
     * (IPv4 in dotted decimal or IPv6 in hex string)
     *  example IPv4: 192.168.0.2
     *  example IPv6: ff02::c3d0:42d2:bb8%wlp4s0 (the interface is specified after %)
     * @param port The port to send SendProto data on
     */
    ThreadedProtoMulticastSender(const std::string& ip_address, unsigned short port);

    ~ThreadedProtoMulticastSender();

    /**
     * Sends a protobuf message over the initialized multicast group and port
     * This function returns after the message has been sent.
     *
     * @param message The protobuf message to send over the multicast group
     */
    void sendProto(const SendProto& message);

   private:
    // The io_service that will be used to service all network requests
    boost::asio::io_service io_service;
    ProtoMulticastSender<SendProto> multicast_sender;
    // The thread running the io_service in the background. This thread will run for the
    // entire lifetime of the class
    std::thread io_service_thread;
};

#include "software/networking/threaded_proto_multicast_sender.tpp"
