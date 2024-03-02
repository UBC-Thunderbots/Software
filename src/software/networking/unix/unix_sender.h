#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>
#include <type_traits>

class UnixSender
{
   public:
    /**
     * Creates a UnixSender that sends a string over the unix socket path.
     *
     * @param io_service The io_service to use to service outgoing SendProto data
     * @param unix_socket_path The path to the unix socket
     */
    UnixSender(boost::asio::io_service& io_service, const std::string& unix_socket_path);
    ~UnixSender();

    /**
     * Sends the string over the unix socket
     *
     * @param message The string message to send
     */
    void sendString(const std::string& message);

   private:
    // A unix socket to send data over
    boost::asio::local::datagram_protocol::socket socket_;

    // The endpoint for the receiver
    boost::asio::local::datagram_protocol::endpoint receiver_endpoint_;
    std::string unix_socket_path_;

    // Failed to send log throttling
    const unsigned MAX_SEND_FAILURES_BEFORE_LOG = 100;
    unsigned log_counter                        = 0;
};
