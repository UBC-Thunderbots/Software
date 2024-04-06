#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>

#include "software/networking/udp/udp_sender.h"

class ThreadedUdpSender
{
   public:
    /**
     * Creates a UdpSender that sends the sendString over the network on the
     * given address and port.
     *
     * @param ip_address The ip address to send data on
     * (IPv4 in dotted decimal or IPv6 in hex string)
     *  example IPv4: 192.168.0.2
     *  example IPv6: ff02::c3d0:42d2:bb8%wlp4s0 (the interface is specified after %)
     * @param port The port to send sendString data on
     * @param multicast If true, joins the multicast group of given ip_address
     */
    ThreadedUdpSender(const std::string& ip_address, unsigned short port, bool multicast);

    ~ThreadedUdpSender();

    /**
     * Sends a string message to the initialized ip address and port
     * This function returns after the message has been sent.
     *
     * @param message The string message to send
     */
    void sendString(const std::string& message);

   private:
    // The io_service that will be used to service all network requests
    boost::asio::io_service io_service;
    UdpSender udp_sender;
    // The thread running the io_service in the background. This thread will run for the
    // entire lifetime of the class
    std::thread io_service_thread;
};
