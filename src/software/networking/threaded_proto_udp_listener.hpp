#pragma once

#include "software/networking/proto_udp_listener.hpp"

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

    void close();

   private:
    // The io_service that will be used to service all network requests
    boost::asio::io_service io_service;
    // The thread running the io_service in the background. This thread will run for the
    // entire lifetime of the class
    std::thread io_service_thread;
    std::function<void(ReceiveProtoT)> receive_callback_;
    ProtoUdpListener<ReceiveProtoT> udp_listener;
};

template <class ReceiveProtoT>
ThreadedProtoUdpListener<ReceiveProtoT>::ThreadedProtoUdpListener(
    const std::string& ip_address, const unsigned short port,
    std::function<void(ReceiveProtoT)> receive_callback, bool multicast)
    : io_service(),
      udp_listener(io_service, ip_address, port, receive_callback, multicast)
{
    // start the thread to run the io_service in the background
    io_service_thread = std::thread([this]() { io_service.run(); });
}

template <class ReceiveProtoT>
ThreadedProtoUdpListener<ReceiveProtoT>::ThreadedProtoUdpListener(
    const unsigned short port, std::function<void(ReceiveProtoT)> receive_callback)
    : io_service(), udp_listener(io_service, port, receive_callback)
{
    // start the thread to run the io_service in the background
    io_service_thread = std::thread([this]() { io_service.run(); });
}

template <class ReceiveProtoT>
ThreadedProtoUdpListener<ReceiveProtoT>::~ThreadedProtoUdpListener()
{
    close();
}


template <class ReceiveProtoT>
void ThreadedProtoUdpListener<ReceiveProtoT>::close()
{
    udp_listener.close();

    // Stop the io_service. This is safe to call from another thread.
    // https://stackoverflow.com/questions/4808848/boost-asio-stopping-io-service
    // This MUST be done before attempting to join the thread because otherwise the
    // io_service will not stop and the thread will not join
    io_service.stop();

    // Join the io_service_thread so that we wait for it to exit before destructing the
    // thread object. If we do not wait for the thread to
    // finish executing, it will call
    // `std::terminate` when we deallocate the thread object and kill our whole program
    io_service_thread.join();

}

#include "software/networking/threaded_proto_udp_listener.hpp"
