#pragma once

#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <string>
#include <type_traits>

#include "software/logger/logger.h"

template <class SendProtoT, class ReceiveProtoT>
class ProtoUdpClient
{
   public:
    /**
     * Creates a ProtoUdpClient that sends the SendProtoT over the network, and blocks
     * until a ReceiveProtoT is received, given the address and port.
     *
     * @param io_service The io_service to use to service outgoing SendProtoT data
     * @param ip_address The ip address to send data on
     * (IPv4 in dotted decimal or IPv6 in hex string)
     *  example IPv4: 192.168.0.2
     *  example IPv6: ff02::c3d0:42d2:bb8%wlp4s0 (the interface is specified after %)
     * @param port The port to send SendProtoT data to
     * @param multicast If true, joins the multicast group of given ip_address
     */
    ProtoUdpClient(boost::asio::io_service& io_service, const std::string& ip_address,
                   unsigned short port, bool multicast);

    virtual ~ProtoUdpClient();

    /**
     * Sends a protobuf message to the initialized ip address and port
     * This function returns after the message has been sent.
     *
     * This function is only enabled if SendProtoT is provided
     *
     * @param message The protobuf message to send
     */
    void sendProto(const SendProtoT& message);

    /**
     * Blocks until a ReceiveProtoT has been received
     *
     * This function is only enabled if ReceiveProtoT is provided
     *
     * @returns ReceiveProtoT
     */
    ReceiveProtoT receiveProto();

    /**
     * Send a proto and block until we hear a response from the server
     *
     * This function is only enabled if ReceiveProtoT and SendProtoT are provided
     *
     * @param message The protobuf message to send
     * @return ReceiveProtoT
     */
    ReceiveProtoT request(const SendProtoT& message);

   private:
    // A UDP socket to send data over
    boost::asio::ip::udp::socket socket_;

    // The endpoint
    boost::asio::ip::udp::endpoint endpoint;

    // Buffer to hold serialized protobuf data
    std::string data_buffer;

    static constexpr unsigned int MAX_BUFFER_LENGTH = 9000;
    std::array<char, MAX_BUFFER_LENGTH> raw_received_data_;
};

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
ReceiveProtoT ProtoUdpClient<SendProtoT, ReceiveProtoT>::receiveProto()
{
    size_t num_bytes_received = socket_.receive_from(
        boost::asio::buffer(raw_received_data_, MAX_BUFFER_LENGTH), endpoint);
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
    socket_.close();
}
