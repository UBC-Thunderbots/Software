#include "software/networking/unix_sender.h"

#include <iostream>

static unsigned MAX_SEND_FAILURES_BEFORE_LOG = 1000;

UnixSender::UnixSender(boost::asio::io_service& io_service,
                       const std::string& unix_socket_path)
    : socket_(io_service)
{
    receiver_endpoint_ =
        boost::asio::local::datagram_protocol::endpoint(unix_socket_path);
    unix_socket_path_ = unix_socket_path;
    socket_.open();
}

void UnixSender::sendString(const std::string& message)
{
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
            // So we have to just log to cerr
            std::cerr << "Unix Socket Send Failure for " << unix_socket_path_
                      << " Make sure the listener is running ->" << ex.what()
                      << std::endl;
            log_counter = 0;
        }
    }
}

UnixSender::~UnixSender()
{
    socket_.close();
}
