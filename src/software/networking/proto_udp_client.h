#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>

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

#include "software/networking/proto_udp_client.tpp"
