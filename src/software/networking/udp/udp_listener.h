#pragma once

#include <boost/asio.hpp>

typedef std::function<void(const char*, const size_t&)> ReceiveCallback;

/**
 * Creates a UDP listener that can listen on a given port and interface.
 */
class UdpListener
{
   public:
    /**
     * Creates a UDP listener.
     *
     * @throws TbotsNetworkException if the listener could not be created
     *
     * @param io_service The service thread to use for the network communication resource
     * @param ip_address If multicast is true, this address is the multicast group to
     * join. Otherwise, this is the IP address of the local interface to listen on
     * @param port The port to listen on
     * @param interface The networking interface to listen on
     * @param multicast If true, the listener will join the multicast group given by
     * `ip_address`, otherwise it will listen on the local interface given by `ip_address`
     * and `interface`
     * @param receive_callback The callback to call when a new message is received
     */
    UdpListener(boost::asio::io_service& io_service, const std::string& ip_address,
                unsigned short port, const std::string& interface, bool multicast,
                ReceiveCallback receive_callback);

    /**
     * Creates a UDP listener that listens on the given port on all interfaces.
     *
     * A user must check the user-provided error string to see if the listener
     * construction was successful.
     *
     * @throws TbotsNetworkException if the listener could not be created
     *
     * @param io_service The service thread to use for the network communication resource
     * @param port The port to listen on
     * @param receive_callback The callback to call when a new message is received
     */
    UdpListener(boost::asio::io_service& io_service, const unsigned short port,
                ReceiveCallback receive_callback);

    /**
     * Destructor.
     */
    virtual ~UdpListener();

    /**
     * Cleans up all associated networking resources with this listener.
     */
    void close();

   private:
    /**
     * Handles the reception of data from the network as well as any errors that may occur
     * before calling the user-provided callback.
     */
    void handleDataReception(const boost::system::error_code& error,
                             std::size_t bytes_transferred);

    /**
     * Starts listening for data on the socket.
     */
    void startListen();

    /**
     * Sets up multicast for the given ip_address and listen_interface
     *
     * @throws TbotsNetworkException if the multicast group could not be joined
     *
     * @param ip_address The ip address of the multicast group to join
     * @param listen_interface The interface to listen on
     */
    void setupMulticast(const boost::asio::ip::address& ip_address,
                        const std::string& listen_interface);

    // The maximum buffer length for the raw data received from the network
    static constexpr unsigned int MAX_BUFFER_LENGTH = 9000;

    // Whether this listener should continue running
    bool running_;

    // The raw data received from the network
    std::array<char, MAX_BUFFER_LENGTH> raw_received_data_;

    // A UDP socket to receive data on
    boost::asio::ip::udp::socket socket_;

    // The endpoint for the sender
    boost::asio::ip::udp::endpoint sender_endpoint_;

    // Callback once a new message is received
    ReceiveCallback receive_callback_;
};
