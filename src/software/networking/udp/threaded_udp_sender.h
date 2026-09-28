#pragma once

#include <boost/asio.hpp>
#include <memory>
#include <string>
#include <utility>

#include "software/networking/udp/threaded_io_context.h"
#include "software/networking/udp/udp_sender.h"

class ThreadedUdpSender
{
   public:
    /**
     * Creates a UdpSender that sends the sendString over the network on the
     * given address and port.
     *
     * @throws TbotsNetworkException if the multicast group could not be joined if the
     * multicast option is requested
     *
     * @param io_context The shared service used to process asynchronous send operations
     * @param ip_address The ip address to send data on
     * (IPv4 in dotted decimal or IPv6 in hex string)
     *  example IPv4: 192.168.0.2
     *  example IPv6: ff02::c3d0:42d2:bb8%wlp4s0 (the interface is specified after %)
     * @param port The port to send sendString data on
     * @param interface The interface to send data on
     * @param multicast If true, joins the multicast group of given ip_address
     */
    ThreadedUdpSender(std::shared_ptr<ThreadedIoContext> io_context,
                      const std::string& ip_address, unsigned short port,
                      const std::string& interface, bool multicast);

    /**
     * Destructor releases this sender's ownership of the shared io_context.
     */
    ~ThreadedUdpSender() = default;

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
     * Sends a string message to the initialized ip address and port
     * This function returns after the message has been sent.
     *
     * @param message The string message to send
     * @param async If true, the message will be sent asynchronously otherwise it will be
     * send synchronously
     */
    void sendString(const std::string& message, bool async = false);

   private:
    // Keeps the shared service alive while the UDP socket exists.
    std::shared_ptr<ThreadedIoContext> io_context_;

    // The UdpSender that will be used to send data over the network
    UdpSender udp_sender_;
};
