#pragma once

#include <memory>
#include <utility>

#include "software/networking/udp/proto_udp_listener.hpp"
#include "software/networking/udp/threaded_io_context.h"

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
     * @throws TbotsNetworkException if we detect an issue with setting up this listener
     *
     * @param io_context The shared service used to process receive operations
     * @param ip_address The ip address on which to listen for the given ReceiveProtoT
     * packets (IPv4 in dotted decimal or IPv6 in hex string) example IPv4: 192.168.0.2
     *  example IPv6: ff02::c3d0:42d2:bb8%wlp4s0
     * @param port The port on which to listen for ReceiveProtoT packets
     * @param interface The interface on which to listen for ReceiveProtoT packets
     * @param receive_callback The function to run for every ReceiveProtoT packet received
     * from the network
     * @param multicast If true, joins the multicast group of given ip_address
     */
    ThreadedProtoUdpListener(std::shared_ptr<ThreadedIoContext> io_context,
                             const std::string& ip_address, unsigned short port,
                             const std::string& interface,
                             std::function<void(ReceiveProtoT)> receive_callback,
                             bool multicast);

    /**
     * Creates a ThreadedProtoUdpListener that will listen for ReceiveProtoT packets
     * from the network on any local address with given port. For every ReceiveProtoT
     * packet received, the receive_callback will be called to perform any operations
     * desired by the caller. This constructor should not be used for multicast
     * communication.
     *
     * @throws TbotsNetworkException if we detect an issue with setting up this listener
     *
     * @param io_context The shared service used to process receive operations
     * @param port The port on which to listen for ReceiveProtoT packets
     * @param interface The interface on which to listen for ReceiveProtoT packets
     * @param receive_callback The function to run for every ReceiveProtoT packet received
     * from the network
     */
    ThreadedProtoUdpListener(std::shared_ptr<ThreadedIoContext> io_context,
                             unsigned short port,
                             std::function<void(ReceiveProtoT)> receive_callback);

    /**
     * Closes this listener's socket without stopping the shared io_context.
     * The shared ThreadedIoContext remains available to service other UDP objects.
     */
    void close();

    /**
     * Destructor closes the socket and releases this listener's service ownership.
     * It does not stop or join the shared io_context thread.
     */
    ~ThreadedProtoUdpListener();


   private:
    // Keeps the shared service alive while the UDP socket exists. The service owns the
    // io_context and the single thread that runs it for all shared UDP objects.
    std::shared_ptr<ThreadedIoContext> io_context_;
    std::function<void(ReceiveProtoT)> receive_callback_;
    ProtoUdpListener<ReceiveProtoT> udp_listener_;
};

template <class ReceiveProtoT>
ThreadedProtoUdpListener<ReceiveProtoT>::ThreadedProtoUdpListener(
    std::shared_ptr<ThreadedIoContext> io_context, const std::string& ip_address,
    const unsigned short port, const std::string& interface,
    std::function<void(ReceiveProtoT)> receive_callback, bool multicast)
    : io_context_(std::move(io_context)),
      receive_callback_(std::move(receive_callback)),
      udp_listener_(io_context_->getIoContext(), ip_address, port, interface,
                    receive_callback_, multicast)
{
}

template <class ReceiveProtoT>
ThreadedProtoUdpListener<ReceiveProtoT>::ThreadedProtoUdpListener(
    std::shared_ptr<ThreadedIoContext> io_context, const unsigned short port,
    std::function<void(ReceiveProtoT)> receive_callback)
    : io_context_(std::move(io_context)),
      receive_callback_(std::move(receive_callback)),
      udp_listener_(io_context_->getIoContext(), port, receive_callback_)
{
}

template <class ReceiveProtoT>
ThreadedProtoUdpListener<ReceiveProtoT>::~ThreadedProtoUdpListener()
{
    close();
}


template <class ReceiveProtoT>
void ThreadedProtoUdpListener<ReceiveProtoT>::close()
{
    udp_listener_.close();
}
