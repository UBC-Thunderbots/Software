#pragma once

#include "software/logger/logger.h"
#include "software/networking/proto_udp_listener.h"
#include "software/util/typename/typename.h"

template <class ReceiveProtoT>
ProtoUdpListener<ReceiveProtoT>::ProtoUdpListener(boost::asio::io_service& io_service,
                                                  const std::string& ip_address,
                                                  const unsigned short port,
                                                  bool multicast)
    : socket_(io_service)
{
    boost::asio::ip::udp::endpoint listen_endpoint(
        boost::asio::ip::make_address(ip_address), port);

    // We want to be able to send and receive on the same port/address so we enable those socket options. 
    // Taken from here: https://gist.github.com/yueyoum/3cbb3b51e7306c7abf1f
    size_t one = 1;
    setsockopt(socket_.native_handle(), SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &one, sizeof(one));
    socket_.open(listen_endpoint.protocol());

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
}

template <class ReceiveProtoT>
ProtoUdpListener<ReceiveProtoT>::ProtoUdpListener(boost::asio::io_service& io_service,
                                                  const unsigned short port)
    : socket_(io_service)
{
    boost::asio::ip::udp::endpoint listen_endpoint(boost::asio::ip::udp::v6(), port);

    // Explicitly set the v6_only option to be false to accept both ipv4 and ipv6 packets
    socket_.set_option(boost::asio::ip::v6_only(false));
    size_t one = 1;
    setsockopt(socket_.native_handle(), SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &one, sizeof(one));
    socket_.open(listen_endpoint.protocol());

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
}

template <class ReceiveProtoT>
ReceiveProtoT ProtoUdpListener<ReceiveProtoT>::receiveProto()
{
    size_t num_bytes_received =
        socket_.receive(boost::asio::buffer(raw_received_data_, MAX_BUFFER_LENGTH));
    auto packet_data = ReceiveProtoT();

    if (num_bytes_received > 0)
    {
        packet_data.ParseFromArray(raw_received_data_.data(),
                                   static_cast<int>(num_bytes_received));
    }
    else
    {
        LOG(WARNING)
            << "An unknown network error occurred when attempting to receive ReceiveProtoT Data. The error code is "
            << num_bytes_received << std::endl;
    }

    if (num_bytes_received > MAX_BUFFER_LENGTH)
    {
        LOG(WARNING)
            << "num_bytes_received > MAX_BUFFER_LENGTH, "
            << "which means that the receive buffer is full and data loss has potentially occurred. "
            << "Consider increasing MAX_BUFFER_LENGTH";
    }

    return packet_data;
}

template <class ReceiveProtoT>
ProtoUdpListener<ReceiveProtoT>::~ProtoUdpListener()
{
    socket_.close();
}
