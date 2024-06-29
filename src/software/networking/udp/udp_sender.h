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
     * Callers must ensure that error is not set before using this object.
     *
     * @param io_service The io_service to use to service outgoing SendString data
     * @param ip_address The ip address to send data on
     * (IPv4 in dotted decimal or IPv6 in hex string)
     *  example IPv4: 192.168.0.2
     *  example IPv6: ff02::c3d0:42d2:bb8%wlp4s0 (the interface is specified after %)
     * @param port The port to send SendString data to
     * @param multicast If true, joins the multicast group of given ip_address
     * @param error A user-provided optional string to store any errors that occur
     */
    UdpSender(boost::asio::io_service& io_service, const std::string& ip_address,
              unsigned short port, const std::string& interface, bool multicast,
              std::optional<std::string>& error);

    ~UdpSender();

    /**
     * Set up multicast for the given multicast ip address and interface
     *
     * Any errors during setup will be stored in the error string
     *
     * @param ip_address The multicast ip address to join
     * @param interface The interface to join the multicast group on
     * @param error A user-provided optional string to store any errors that occur
     */
    void setupMulticast(const boost::asio::ip::address& ip_address,
                        const std::string& interface, std::optional<std::string>& error);

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
};
