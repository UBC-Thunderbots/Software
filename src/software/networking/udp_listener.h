#pragma once

#include <boost/asio.hpp>

typedef std::function<void(const char*, const std::size_t&)> ReceiveCallback;

class UdpListener
{
    public:
        UdpListener(boost::asio::io_service& io_service, const std::string& ip_address,
                    unsigned short port, bool multicast, ReceiveCallback receive_callback);

        UdpListener(boost::asio::io_service& io_service, const unsigned short port, ReceiveCallback receive_callback);

        virtual ~UdpListener();

    private:
        void close();

        void handleDataReception(const boost::system::error_code& error, std::size_t bytes_transferred);

        void startListen();

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
