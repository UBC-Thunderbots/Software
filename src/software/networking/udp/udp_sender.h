#pragma once

#include <boost/asio.hpp>
#include <string>

class UdpSender
{
   public:
    /**
     * Creates a UdpSender that sends strings over the network to the
     * given address and port.
     *
     * @throws TbotsNetworkException if the listener could not be created
     *
     * @param io_service The io_service to use to service outgoing SendString data
     * @param ip_address The ip address to send data on
     * (IPv4 in dotted decimal or IPv6 in hex string)
     *  example IPv4: 192.168.0.2
     *  example IPv6: ff02::c3d0:42d2:bb8
     * @param port The port to send SendString data to
     * @param interface The interface to send data on
     * @param multicast If true, joins the multicast group of given ip_address
     */
    UdpSender(boost::asio::io_service& io_service, const std::string& ip_address,
              unsigned short port, const std::string& interface, bool multicast);

    /**
     * Destructor
     */
    ~UdpSender();

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
     * Get the port that this sender is sending to
     *
     * @return The port as an unsigned short
     */

    /**
     * Sends a string message to the initialized ip address and port
     * This function returns after the message has been sent.
     *
     * @param message The string message to send
     */
    void sendString(const std::string& message);

    /**
     * Sends a string message to the initialized ip address and port _asynchronously_
     * This function returns immediately.
     *
     * @param message The string message to send
     */
    void sendStringAsync(const std::string& message);

   private:
    /**
     * Set up multicast for the given multicast ip address and interface
     *
     * @throws TbotsNetworkException if the multicast group could not be joined
     *
     * @param ip_address The multicast ip address to join
     * @param interface The interface to join the multicast group on
     */
    void setupMulticast(const boost::asio::ip::address& ip_address,
                        const std::string& interface);

    // A UDP socket to send data over
    boost::asio::ip::udp::socket socket_;

    // The endpoint for the receiver
    boost::asio::ip::udp::endpoint receiver_endpoint;

    // The interface to send data on
    std::string interface_;

    // The IP address to send data to (IPv4 in dotted decimal or IPv6 in hex string)
    std::string ip_address_;
};
