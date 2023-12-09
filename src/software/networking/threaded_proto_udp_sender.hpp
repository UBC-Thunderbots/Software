#pragma once

#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <string>

#include "software/networking/udp_sender.h"

template <class SendProto>
class ThreadedProtoUdpSender
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

    std::string data_buffer;
    UdpSender udp_sender;

    // The thread running the io_service in the background. This thread will run for the
    // entire lifetime of the class
    std::thread io_service_thread;
};

template <class SendProtoT>
ThreadedProtoUdpSender<SendProtoT>::ThreadedProtoUdpSender(const std::string& ip_address,
                                                           const unsigned short port,
                                                           bool multicast)
    : io_service(),
      udp_sender(io_service, ip_address, port, multicast),
      io_service_thread([this]() { io_service.run(); })
{
}

template <class SendProtoT>
ThreadedProtoUdpSender<SendProtoT>::~ThreadedProtoUdpSender()
{
    // Stop the io_service. This is safe to call from another thread.
    // https://stackoverflow.com/questions/4808848/boost-asio-stopping-io-service
    // This MUST be done before attempting to join the thread because otherwise the
    // io_service will not stop and the thread will not join
    io_service.stop();

    // Join the io_service_thread so that we wait for it to exit before destructing the
    // thread object. If we do not wait for the thread to finish executing, it will call
    // `std::terminate` when we deallocate the thread object and kill our whole program
    io_service_thread.join();
}

template <class SendProtoT>
void ThreadedProtoUdpSender<SendProtoT>::sendProto(const SendProtoT& message)
{
    message.SerializeToString(&data_buffer);
    udp_sender.sendString(data_buffer);
}
