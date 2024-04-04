#include "software/networking/unix/unix_sender.h"

#include <iostream>

#include "software/constants.h"

UnixSender::UnixSender(boost::asio::io_service& io_service,
                       const std::string& unix_socket_path)
    : socket_(io_service)
{
    receiver_endpoint_ =
        boost::asio::local::datagram_protocol::endpoint(unix_socket_path);
    unix_socket_path_ = unix_socket_path;
    socket_.open();

    boost::asio::local::datagram_protocol::socket::send_buffer_size option(
        UNIX_BUFFER_SIZE);
    socket_.set_option(option);
}

void UnixSender::sendString(const std::string& message)
{
    // If the listener isn't running already, the send_to call
    // will throw an exception. We need to catch it and log so
    // we don't crash and burn if programs are launched out of order.
    try
    {
        socket_.send_to(boost::asio::buffer(message, message.length()),
                        receiver_endpoint_);
    }
    catch (const boost::system::system_error& ex)
    {
        log_counter++;
        if (log_counter > MAX_SEND_FAILURES_BEFORE_LOG)
        {
            // NOTE: g3log relies on the UnixSender so we can't
            // use g3log here without creating a circular dependency.
            // So we have to just log to cerr with some ANSI escape chars
            // to print in yellow.
            std::cerr << "\033[33m Unix Socket Send Failure for " << unix_socket_path_
                      << ". Make sure the listener is running : " << ex.what() << "\033[m"
                      << std::endl;
            log_counter = 0;
        }
    }
}

UnixSender::~UnixSender()
{
    socket_.close();
}
