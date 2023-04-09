#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>

class UdpSender
{
   public:
    /**
     * Creates a UdpSender that sends strings over the network to the
     * given address and port.
     *
     * The UdpSender will be assigned a random available port on creation
     * so it doesn't conflict with anything else on the system.
     *
     * @param io_service The io_service to use to service outgoing SendString data
     * @param ip_address The ip address to send data on
     * (IPv4 in dotted decimal or IPv6 in hex string)
     *  example IPv4: 192.168.0.2
     *  example IPv6: ff02::c3d0:42d2:bb8%wlp4s0 (the interface is specified after %)
     * @param port The port to send SendString data to
     * @param multicast If true, joins the multicast group of given ip_address
     */
    UdpSender(boost::asio::io_service& io_service, const std::string& ip_address,
              unsigned short port, bool multicast);

    virtual ~UdpSender();

    /**
     * Sends a string message to the initialized ip address and port
     * This function returns after the message has been sent.
     *
     * @param message The string message to send
     */
    void sendString(const std::string& message);

   private:
    // A UDP socket to send data over
    boost::asio::ip::udp::socket socket_;

    // The endpoint for the receiver
    boost::asio::ip::udp::endpoint receiver_endpoint;

    int i = 0;
};
