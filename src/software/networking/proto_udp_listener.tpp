#pragma once

#include "software/logger/logger.h"
#include "software/networking/proto_udp_listener.h"
#include "software/util/typename/typename.h"

template <class ReceiveProtoT>
ProtoUdpListener<ReceiveProtoT>::ProtoUdpListener(
    boost::asio::io_service& io_service, const std::string& ip_address,
    const unsigned short port, std::function<void(ReceiveProtoT&)> receive_callback,
    bool multicast)
    : socket_(io_service), receive_callback(receive_callback)
{
    boost::asio::ip::udp::endpoint listen_endpoint(
        boost::asio::ip::make_address(ip_address), port);
    socket_.open(listen_endpoint.protocol());
    socket_.set_option(boost::asio::socket_base::reuse_address(true));
    try
    {
        socket_.bind(listen_endpoint);
    }
    catch (const boost::exception& ex)
    {
        LOG(FATAL) << "UdpListener: There was an issue binding the socket to "
                      "the listen_endpoint when trying to connect to the "
                      "address. This may be due to another instance of the "
                      "UdpListener running and using the port already. "
                      "(ip = "
                   << ip_address << ", port = " << port << ")" << std::endl;
    }

    if (multicast)
    {
        // Join the multicast group.
        socket_.set_option(boost::asio::ip::multicast::join_group(
            boost::asio::ip::address::from_string(ip_address)));
    }

    startListen();
}

template <class ReceiveProtoT>
ProtoUdpListener<ReceiveProtoT>::ProtoUdpListener(
    boost::asio::io_service& io_service, const unsigned short port,
    std::function<void(ReceiveProtoT&)> receive_callback)
    : socket_(io_service), receive_callback(receive_callback)
{
    boost::asio::ip::udp::endpoint listen_endpoint(boost::asio::ip::udp::v6(), port);
    socket_.open(listen_endpoint.protocol());
    // Explicitly set the v6_only option to be false to accept both ipv4 and ipv6 packets
    socket_.set_option(boost::asio::ip::v6_only(false));
    try
    {
        socket_.bind(listen_endpoint);
    }
    catch (const boost::exception& ex)
    {
        LOG(FATAL) << "UdpListener: There was an issue binding the socket to "
                      "the listen_endpoint when trying to connect to the "
                      "address. This may be due to another instance of the "
                      "UdpListener running and using the port already. "
                      "(port = "
                   << port << ")" << std::endl;
    }

    startListen();
}

template <class ReceiveProtoT>
void ProtoUdpListener<ReceiveProtoT>::startListen()
{
    // Start listening for data asynchronously
    // See here for a great explanation about asynchronous operations:
    // https://stackoverflow.com/questions/34680985/what-is-the-difference-between-asynchronous-programming-and-multithreading
    socket_.async_receive_from(boost::asio::buffer(raw_received_data_, MAX_BUFFER_LENGTH),
                               sender_endpoint_,
                               boost::bind(&ProtoUdpListener::handleDataReception, this,
                                           boost::asio::placeholders::error,
                                           boost::asio::placeholders::bytes_transferred));
}

template <class ReceiveProtoT>
void ProtoUdpListener<ReceiveProtoT>::handleDataReception(
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

    if (num_bytes_received > MAX_BUFFER_LENGTH)
    {
        LOG(WARNING)
            << "num_bytes_received > MAX_BUFFER_LENGTH, "
            << "which means that the receive buffer is full and data loss has potentially occurred. "
            << "Consider increasing MAX_BUFFER_LENGTH";
    }
}

template <class ReceiveProtoT>
ProtoUdpListener<ReceiveProtoT>::~ProtoUdpListener()
{
    socket_.close();
}
