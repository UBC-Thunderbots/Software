#pragma once

#include <boost/asio.hpp>

typedef std::function<void(const char*, const size_t&)> ReceiveCallback;

class UdpListener
{
   public:
    UdpListener(boost::asio::io_service& io_service,
            const std::string& ip_address,
                unsigned short port,
                const std::string& interface,
                bool multicast,
                ReceiveCallback receive_callback,
                std::optional<std::string>& error);

    UdpListener(boost::asio::io_service& io_service,
                const unsigned short port,
                ReceiveCallback receive_callback,
                std::optional<std::string>& error);

    virtual ~UdpListener();

   private:
    void close();

    void handleDataReception(const boost::system::error_code& error,
                             std::size_t bytes_transferred);

    void startListen();

    /**
     * Sets up multicast for the given ip_address and listen_interface
     *
     * Any errors during setup will be stored in the error string
     *
     * @param ip_address The ip address of the multicast group to join
     * @param listen_interface The interface to listen on
     * @param error A user-provided optional string to store any error messages
     */
    void setupMulticast(const boost::asio::ip::address& ip_address,
                        const std::string& listen_interface,
                        std::optional<std::string>& error);

    static constexpr unsigned int MAX_BUFFER_LENGTH = 9000;

    bool running_;

    std::array<char, MAX_BUFFER_LENGTH> raw_received_data_;

    // A UDP socket to receive data on
    boost::asio::ip::udp::socket socket_;

    // The endpoint for the sender
    boost::asio::ip::udp::endpoint sender_endpoint_;

    // Callback once a new message is received
    ReceiveCallback receive_callback_;
};
