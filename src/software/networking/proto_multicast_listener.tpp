#pragma once

#include "software/logger/logger.h"

template <class ReceiveProto>
ProtoMulticastListener<ReceiveProto>::ProtoMulticastListener(
    boost::asio::io_service& io_service, const std::string& ip_address,
    const unsigned short port, std::function<void(ReceiveProto)> receive_callback)
    : socket_(io_service), receive_callback(receive_callback)
{
    boost::asio::ip::udp::endpoint listen_endpoint(
        boost::asio::ip::make_address(ip_address), port);
    socket_.open(listen_endpoint.protocol());
    try
    {
        socket_.bind(listen_endpoint);
    }
    catch (const boost::exception& ex)
    {
        LOG(FATAL) << "There was an issue binding the socket to the endpoint when"
                      "trying to connect to the multicast address. This may"
                      "be due to another instance of the ProtoMulicastListener running"
                      "and using the port already"
                   << std::endl;
    }

    // Join the multicast group.
    socket_.set_option(boost::asio::ip::multicast::join_group(
        boost::asio::ip::address::from_string(ip_address)));

    // Start listening for data asynchronously
    // See here for a great explanation about asynchronous operations:
    // https://stackoverflow.com/questions/34680985/what-is-the-difference-between-asynchronous-programming-and-multithreading
    socket_.async_receive_from(boost::asio::buffer(raw_received_data_, max_buffer_length),
                               sender_endpoint_,
                               boost::bind(&ProtoMulticastListener::handleDataReception,
                                           this, boost::asio::placeholders::error,
                                           boost::asio::placeholders::bytes_transferred));
}

template <class ReceiveProto>
void ProtoMulticastListener<ReceiveProto>::handleDataReception(
    const boost::system::error_code& error, size_t num_bytes_received)
{
    if (!error)
    {
        auto packet_data = ReceiveProto();
        packet_data.ParseFromArray(raw_received_data_.data(),
                                   static_cast<int>(num_bytes_received));
        receive_callback(packet_data);

        // Once we've handled the data, start listening again
        socket_.async_receive_from(
            boost::asio::buffer(raw_received_data_, max_buffer_length), sender_endpoint_,
            boost::bind(&ProtoMulticastListener::handleDataReception, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));
    }
    else
    {
        // Start listening again to receive the next data
        socket_.async_receive_from(
            boost::asio::buffer(raw_received_data_, max_buffer_length), sender_endpoint_,
            boost::bind(&ProtoMulticastListener::handleDataReception, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));

        LOG(WARNING)
            << "An unknown network error occurred when attempting to receive ReceiveProto Data. The boost system error code is "
            << error << std::endl;
    }

    if (num_bytes_received >= max_buffer_length)
    {
        LOG(WARNING)
            << "num_bytes_received >= max_buffer_length, "
            << "which means that the receive buffer is full and data loss has potentially occurred. "
            << "Consider increasing max_buffer_length";
    }
}

template <class ReceiveProto>
ProtoMulticastListener<ReceiveProto>::~ProtoMulticastListener()
{
    socket_.close();
}
