#pragma once

#include "software/networking/proto_udp_client.h"

/**
 * A threaded listener that receives serialized ReceiveProtoT Proto's over the network
 */
template <class ReceiveProtoT>
class ThreadedProtoUdpListener
{
   public:
    /**
     * Creates a ThreadedProtoUdpListener that will listen for ReceiveProtoT packets
     * from the network of given address and port. For every
     * ReceiveProtoT packet received, the receive_callback will be called to perform any
     * operations desired by the caller.
     *
     * @param ip_address The ip address on which to listen for the given ReceiveProtoT
     * packets (IPv4 in dotted decimal or IPv6 in hex string) example IPv4: 192.168.0.2
     *  example IPv6: ff02::c3d0:42d2:bb8%wlp4s0 (the interface is specified after %)
     * @param port The port on which to listen for ReceiveProtoT packets
     * @param receive_callback The function to run for every ReceiveProtoT packet received
     * from the network
     * @param multicast If true, joins the multicast group of given ip_address
     */
    ThreadedProtoUdpListener(const std::string& ip_address, unsigned short port,
                             std::function<void(ReceiveProtoT)> receive_callback,
                             bool multicast);

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

    // The thread running the io_service and receiving packets in the background.
    // This thread will run for the entire lifetime of the class
    std::thread networking_thread;

    // The function to call on every received packet of ReceiveProtoT data
    std::function<void(ReceiveProtoT)> receive_callback;

    // The threaded proto udp listener will never receive any messages, we just
    // use the ReceiveProtoT type as a placeholder
    ProtoUdpClient<ReceiveProtoT, ReceiveProtoT> udp_listener;

    std::atomic_bool in_destructor;
};

#include "software/networking/threaded_proto_udp_listener.tpp"
