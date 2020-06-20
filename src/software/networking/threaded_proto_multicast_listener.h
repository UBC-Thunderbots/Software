#pragma once

#include <boost/asio.hpp>
#include <string>
#include <thread>

#include "software/networking/proto_multicast_listener.h"

template <class ReceiveProto>
class ThreadedProtoMulticastListener
{
   public:
    /**
     * Creates a ThreadedProtoMulticastListener that will listen for ReceiveProto packets
     * from the network on the given address and port. For every ReceiveProto packet
     * received, the receive_callback will be called to perform any operations desired by
     * the caller.
     *
     * @param ip_address The ip address of the multicast group on which to listen for
     * the given ReceiveProto packets (IPv4 in dotted decimal or IPv6 in hex string)
     *  example IPv4: 192.168.0.2
     *  example IPv6: ff02::c3d0:42d2:bb8%wlp4s0 (the interface is specified after %)
     * @param port The port on which to listen for ReceiveProto packets
     * @param receive_callback The function to run for every ReceiveProto packet received
     * from the network
     */
    ThreadedProtoMulticastListener(const std::string& ip_address, unsigned short port,
                                   std::function<void(ReceiveProto)> receive_callback);

    ~ThreadedProtoMulticastListener();

   private:
    // The io_service that will be used to serivce all network requests
    boost::asio::io_service io_service;
    // The thread running the io_service in the background. This thread will run for the
    // entire lifetime of the class
    std::thread io_service_thread;
    ProtoMulticastListener<ReceiveProto> multicast_listener;
};

#include "software/networking/threaded_proto_multicast_listener.tpp"
