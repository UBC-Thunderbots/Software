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

    /*
     * NOTE: The following functions utilize SFINAE (substitution failure is not an error)
     *
     * We want to be able to re-use these functions across all networking libraries,
     * but we want to be able to compile out functions that that user doesn't need
     * to prevent unintended use (we don't want "listeners" to be able to send)
     * (the user should use the client library in that case)
     *
     * std::enable_if allows us to remove the function from the overload set if the
     * substitution fails, allowing us to "turn on" and "turn off" different functions,
     * based on the template arguments.
     *
     * https://stackoverflow.com/questions/6972368/stdenable-if-to-conditionally-compile-a-member-function
     */

    /**
     * Sends a protobuf message to the initialized ip address and port
     * This function returns after the message has been sent.
     *
     * This function is only enabled if SendProtoT is provided
     *
     * @param message The protobuf message to send
     */
    template <class = std::enable_if_t<!std::is_same<SendProtoT, void>::value>>
    void sendProto(const SendProtoT& message);

    /**
     * Blocks until a ReceiveProtoT has been received
     *
     * This function is only enabled if ReceiveProtoT is provided
     *
     * @returns ReceiveProtoT
     */
    template <class = std::enable_if_t<!std::is_same<ReceiveProtoT, void>::value>>
    ReceiveProtoT receiveProto();

    /**
     * Send a proto and block until we hear a response from the server
     *
     * This function is only enabled if ReceiveProtoT and SendProtoT are provided
     *
     * @param message The protobuf message to send
     * @return ReceiveProtoT
     */
    template <class = std::enable_if_t<!std::is_same<ReceiveProtoT, void>::value &&
                                       !std::is_same<SendProtoT, void>::value>>
    ReceiveProtoT request(const SendProtoT& message);

   private:
    // A UDP socket to send data over
    boost::asio::ip::udp::socket socket_;

    // The endpoint for the receiver
    boost::asio::ip::udp::endpoint receiver_endpoint;

    // Buffer to hold serialized protobuf data
    std::string data_buffer;

    static constexpr unsigned int MAX_BUFFER_LENGTH = 9000;
    std::array<char, MAX_BUFFER_LENGTH> raw_received_data_;
};

#include "software/networking/proto_udp_client.tpp"
