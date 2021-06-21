#pragma once

#include <chrono>
#include <type_traits>

#include "software/logger/logger.h"

template <class SendProtoT, class ReceiveProtoT>
ProtoUdpClient<SendProtoT, ReceiveProtoT>::ProtoUdpClient(const std::string& ip_address,
                                                          const unsigned short port,
                                                          bool multicast,
                                                          bool enable_timeout)
    : io_service(), socket_(io_service), work(io_service), enable_timeout(enable_timeout)
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
    if (enable_timeout)
    {
        // If timeout is enabled, asynchronous receive function is used.
        // Start the thread to run the io_service in the background.
        io_service_thread = std::thread([this]() { io_service.run(); });
    }
}

template <class SendProtoT, class ReceiveProtoT>
void ProtoUdpClient<SendProtoT, ReceiveProtoT>::sendProto(const SendProtoT& message)
{
    message.SerializeToString(&data_buffer);
    socket_.send_to(boost::asio::buffer(data_buffer), endpoint);
}

template <class SendProtoT, class ReceiveProtoT>
ReceiveProtoT ProtoUdpClient<SendProtoT, ReceiveProtoT>::receiveProto()
{
    size_t num_bytes_received;
    if (enable_timeout)
    {
        std::future<size_t> read_result = socket_.async_receive_from(
            boost::asio::buffer(raw_received_data_, MAX_BUFFER_LENGTH), endpoint,
            boost::asio::use_future);

        // Timeout occurs
        if (read_result.wait_for(std::chrono::milliseconds(100)) ==
            std::future_status::timeout)
        {
            socket_.cancel();
            return ReceiveProtoT();
        }
        else
        {
            num_bytes_received = read_result.get();
        }
    }
    else
    {
        num_bytes_received = socket_.receive_from(
            boost::asio::buffer(raw_received_data_, MAX_BUFFER_LENGTH), endpoint);
    }
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
    if (enable_timeout)
    {
        // Stop the io_service. This is safe to call from another thread.
        // https://stackoverflow.com/questions/4808848/boost-asio-stopping-io-service
        // This MUST be done before attempting to join the thread because otherwise the
        // io_service will not stop and the thread will not join
        io_service.stop();

        // Join the io_service_thread so that we wait for it to exit before destructing
        // the thread object. If we do not wait for the thread to finish executing, it
        // will call `std::terminate` when we deallocate the thread object and kill our
        // whole program
        io_service_thread.join();
    }
    socket_.close();
}
