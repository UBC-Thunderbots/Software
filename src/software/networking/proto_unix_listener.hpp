#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>

#include "software/logger/logger.h"
#include "software/networking/proto_udp_listener.hpp"
#include "software/util/typename/typename.h"

template <class ReceiveProtoT>
class ProtoUnixListener
{
   public:
    /**
     * Creates an ProtoUnixListener that will listen for ReceiveProtoT packets from
     * the network on the multicast group of given address and port. For every
     * ReceiveProtoT packet received, the receive_callback will be called to perform any
     * operations fdesired by the caller
     *
     * @param io_service The io_service to use to service incoming ReceiveProtoT data
     * @param ip_address The ip address of on which to listen for the given ReceiveProtoT
     * packets (IPv4 in dotted decimal or IPv6 in hex string) example IPv4: 192.168.0.2
     *  example IPv6: ff02::c3d0:42d2:bb8%wlp4s0 (the interface is specified after %)
     * @param port The port on which to listen for ReceiveProtoT packets
     * @param receive_callback The function to run for every ReceiveProtoT packet received
     * from the network
     * @param multicast If true, joins the multicast group of given ip_address
     */
    ProtoUnixListener(boost::asio::io_service& io_service, const std::string& ip_address,
                      unsigned short port,
                      std::function<void(ReceiveProtoT&)> receive_callback,
                      bool multicast);

    /**
     * Creates an ProtoUnixListener that will listen for ReceiveProtoT packets from
     * the network on any local address with given port. For every ReceiveProtoT packet
     * received, the receive_callback will be called to perform any operations desired by
     * the caller
     *
     * @param io_service The io_service to use to service incoming ReceiveProtoT data
     * @param port The port on which to listen for ReceiveProtoT packets
     * @param receive_callback The function to run for every ReceiveProtoT packet received
     * from the network
     */
    ProtoUnixListener(boost::asio::io_service& io_service, unsigned short port,
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

    // A UDP socket that we listen on for ReceiveProtoT messages from the network
    boost::asio::local::datagram_protocol::socket socket_;
    // The endpoint for the sender
    boost::asio::local::datagram_protocol::endpoint listen_endpoint_;
    boost::asio::local::datagram_protocol::endpoint endpoint_;

    static constexpr unsigned int MAX_BUFFER_LENGTH = 9000;
    std::array<char, MAX_BUFFER_LENGTH> raw_received_data_;

    // The function to call on every received packet of ReceiveProtoT data
    std::function<void(ReceiveProtoT&)> receive_callback;
};

template <class ReceiveProtoT>
ProtoUnixListener<ReceiveProtoT>::ProtoUnixListener(
    boost::asio::io_service& io_service, const std::string& ip_address,
    const unsigned short port, std::function<void(ReceiveProtoT&)> receive_callback,
    bool multicast)
    : socket_(io_service), receive_callback(receive_callback)
{
    remove(ip_address.c_str());
    listen_endpoint_ = boost::asio::local::datagram_protocol::endpoint(ip_address);
    socket_.open();
    socket_.bind(listen_endpoint_);
    startListen();
}

template <class ReceiveProtoT>
ProtoUnixListener<ReceiveProtoT>::ProtoUnixListener(
    boost::asio::io_service& io_service, const unsigned short port,
    std::function<void(ReceiveProtoT&)> receive_callback)
    : socket_(io_service), receive_callback(receive_callback)
{
}

template <class ReceiveProtoT>
void ProtoUnixListener<ReceiveProtoT>::startListen()
{
    // Start listening for data asynchronously
    // See here for a great explanation about asynchronous operations:
    // https://stackoverflow.com/questions/34680985/what-is-the-difference-between-asynchronous-programming-and-multithreading
    socket_.async_receive_from(boost::asio::buffer(raw_received_data_, MAX_BUFFER_LENGTH),
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
        std::string str(std::begin(raw_received_data_), std::end(raw_received_data_));
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
ProtoUnixListener<ReceiveProtoT>::~ProtoUnixListener()
{
    socket_.close();
}
