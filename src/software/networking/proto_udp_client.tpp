#pragma once

#include <type_traits>

#include "software/logger/logger.h"

template <class SendProtoT, class ReceiveProtoT>
ProtoUdpClient<SendProtoT, ReceiveProtoT>::ProtoUdpClient(
    boost::asio::io_service& io_service, const std::string& ip_address,
    const unsigned short port, bool multicast)
    : socket_(io_service)
{
    boost::asio::ip::address addr = boost::asio::ip::make_address(ip_address);
    endpoint                      = boost::asio::ip::udp::endpoint(addr, port);

    socket_.open(endpoint.protocol());
    socket_.set_option(boost::asio::socket_base::reuse_address(true));

    if (multicast)
    {
        try
        {
            // We need to bind to the multicast port to be able to send/listen to
            // messages. We _should not_ bind to the endpoint in a unicast setting
            // in case the server is running on the same computer (simulators)
            //
            // UDP is connectionless, and will work w/out a bind w/ just sendto and
            // recvfrom
            socket_.bind(boost::asio::ip::udp::endpoint(endpoint));
        }
        catch (const boost::exception& ex)
        {
            LOG(FATAL) << "ProtoUdpClient: There was an issue binding the socket to "
                          "the endpoint when trying to connect to the "
                          "address. This may be due to another instance of the "
                          "ProtoUdpClient running and using the port already. "
                          "(ip = "
                       << ip_address << ", port = " << port << ")" << std::endl;
        }

        socket_.set_option(boost::asio::ip::multicast::join_group(addr));
    }
}

template <class SendProtoT, class ReceiveProtoT>
void ProtoUdpClient<SendProtoT, ReceiveProtoT>::sendProto(const SendProtoT& message)
{
    message.SerializeToString(&data_buffer);
    socket_.send_to(boost::asio::buffer(data_buffer), endpoint);
}

template <class SendProtoT, class ReceiveProtoT>
std::optional<ReceiveProtoT> ProtoUdpClient<SendProtoT, ReceiveProtoT>::receiveProto()
{
    std::future<std::size_t> recv_length = socket_.async_receive_from(
        boost::asio::buffer(raw_received_data_), endpoint, boost::asio::use_future);

    std::future_status status =
        recv_length.wait_for(std::chrono::milliseconds(UDP_RECEIVE_TIMEOUT_MS));
    while (status != std::future_status::ready)
    {
        return std::nullopt;
    }

    size_t num_bytes_received = recv_length.get();
    auto packet_data          = ReceiveProtoT();

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

template <class SendProtoT, class ReceiveProtoT>
ReceiveProtoT ProtoUdpClient<SendProtoT, ReceiveProtoT>::request(
    const SendProtoT& request)
{
    ProtoUdpClient<SendProtoT, ReceiveProtoT>::sendProto(request);
    return ProtoUdpClient<SendProtoT, ReceiveProtoT>::receiveProto();
}

template <class SendProtoT, class ReceiveProtoT>
ProtoUdpClient<SendProtoT, ReceiveProtoT>::~ProtoUdpClient()
{
    socket_.cancel();
    socket_.close();
}
