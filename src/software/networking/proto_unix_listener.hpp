#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>

#include "software/constants.h"
#include "software/logger/logger.h"
#include "software/util/typename/typename.h"

template <class ReceiveProtoT>
class ProtoUnixListener
{
   public:
    /*
     * Listens for packets over the provided unix socket and triggers the
     * receive_callback on a new message.
     *
     * @param io_service The io service
     * @param unix_path The unix path to connect to
     * @param receive_callback The callback to trigger on a new packet
     */
    ProtoUnixListener(boost::asio::io_service& io_service, const std::string& unix_path,
                      std::function<void(ReceiveProtoT&)> receive_callback);

    virtual ~ProtoUnixListener();

   private:
    /**
     * This function is setup as the callback to handle packets received over the network.
     *
     * @param error The error code obtained when receiving the incoming data
     * @param num_bytes_received How many bytes of data were received
     */
    void handleDataReception(const boost::system::error_code& error,
                             size_t num_bytes_received);

    /**
     * Start listening for data
     */
    void startListen();

    std::string unix_path_;

    // A unix socket that we receive packets on
    boost::asio::local::datagram_protocol::socket socket_;

    // The endpoint for the sender
    boost::asio::local::datagram_protocol::endpoint listen_endpoint_;
    std::array<char, UNIX_BUFFER_SIZE> raw_received_data_;

    // The function to call on every received packet of ReceiveProtoT data
    std::function<void(ReceiveProtoT&)> receive_callback;
};

template <class ReceiveProtoT>
ProtoUnixListener<ReceiveProtoT>::ProtoUnixListener(
    boost::asio::io_service& io_service, const std::string& unix_path,
    std::function<void(ReceiveProtoT&)> receive_callback)
    : socket_(io_service), receive_callback(receive_callback)
{
    ::unlink(unix_path.c_str());

    listen_endpoint_ = boost::asio::local::datagram_protocol::endpoint(unix_path);
    unix_path_       = unix_path;


    socket_.open();
    socket_.bind(listen_endpoint_);

    boost::asio::local::datagram_protocol::socket::receive_buffer_size option(
        UNIX_BUFFER_SIZE);
    socket_.set_option(option);

    startListen();
}

template <class ReceiveProtoT>
void ProtoUnixListener<ReceiveProtoT>::startListen()
{
    // Start listening for data asynchronously
    // See here for a great explanation about asynchronous operations:
    // https://stackoverflow.com/questions/34680985/what-is-the-difference-between-asynchronous-programming-and-multithreading
    socket_.async_receive_from(boost::asio::buffer(raw_received_data_, UNIX_BUFFER_SIZE),
                               listen_endpoint_,
                               boost::bind(&ProtoUnixListener::handleDataReception, this,
                                           boost::asio::placeholders::error,
                                           boost::asio::placeholders::bytes_transferred));
}

template <class ReceiveProtoT>
void ProtoUnixListener<ReceiveProtoT>::handleDataReception(
    const boost::system::error_code& error, size_t num_bytes_received)
{
    if (!error)
    {
        auto packet_data = ReceiveProtoT();
        packet_data.ParseFromArray(raw_received_data_.data(),
                                   static_cast<int>(num_bytes_received));
        receive_callback(packet_data);

        // Once we've handled the data, start listening again
        startListen();
    }
    else
    {
        // Start listening again to receive the next data
        startListen();

        LOG(WARNING)
            << "An unknown network error occurred when attempting to receive ReceiveProtoT Data. The boost system error code is "
            << error << std::endl;
    }

    if (num_bytes_received > UNIX_BUFFER_SIZE)
    {
        LOG(WARNING)
            << "num_bytes_received > UNIX_BUFFER_SIZE, "
            << "which means that the receive buffer is full and data loss has potentially occurred. "
            << "Consider increasing UNIX_BUFFER_SIZE";
    }
}

template <class ReceiveProtoT>
ProtoUnixListener<ReceiveProtoT>::~ProtoUnixListener()
{
    socket_.close();
}
