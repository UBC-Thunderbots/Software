#pragma once

#include "software/networking/proto_udp_listener.h"

/**
 * A threaded listener that receives serialized ReceiveProtoT Proto's over the network
 */
template <class ReceiveProtoT>
class ThreadedProtoUdpListener
{
   public:
    /**
     * Creates a ThreadedProtoUdpListener that will listen for ReceiveProtoT packets
     * from the network on the multicast group of given address and port. For every
     * ReceiveProtoT packet received, the receive_callback will be called to perform any
     * operations desired by the caller.
     *
     * @param ip_address The ip address of the multicast group on which to listen for
     * the given ReceiveProtoT packets (IPv4 in dotted decimal or IPv6 in hex string)
     *  example IPv4: 192.168.0.2
     *  example IPv6: ff02::c3d0:42d2:bb8%wlp4s0 (the interface is specified after %)
     * @param port The port on which to listen for ReceiveProtoT packets
     * @param receive_callback The function to run for every ReceiveProtoT packet received
     * from the network
     */
    ThreadedProtoUdpListener(const std::string& ip_address, unsigned short port,
                             std::function<void(ReceiveProtoT)> receive_callback);

    /**
     * Creates a ThreadedProtoUdpListener that will listen for ReceiveProtoT packets
     * from the network on any local address with given port. For every ReceiveProtoT
     * packet received, the receive_callback will be called to perform any operations
     * desired by the caller.
     *
     * @param port The port on which to listen for ReceiveProtoT packets
     * @param receive_callback The function to run for every ReceiveProtoT packet received
     * from the network
     */
    ThreadedProtoUdpListener(unsigned short port,
                             std::function<void(ReceiveProtoT)> receive_callback);

    ~ThreadedProtoUdpListener();

   private:
    // The io_service that will be used to service all network requests
    boost::asio::io_service io_service;
    // The thread running the io_service in the background. This thread will run for the
    // entire lifetime of the class
    std::thread io_service_thread;
    ProtoUdpListener<ReceiveProtoT> udp_listener;
};

#include "software/networking/threaded_proto_udp_listener.tpp"
