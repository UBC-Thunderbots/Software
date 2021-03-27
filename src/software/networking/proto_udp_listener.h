#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>

template <class ReceiveProtoT>
class ProtoUdpListener
{
   public:
    /**
     * Creates an ProtoUdpListener that will listen for ReceiveProtoT packets from
     * the network on the given address and port. For every ReceiveProtoT packet received,
     * the receive_callback will be called to perform any operations desired by the caller
     *
     * @param io_service The io_service to use to service incoming ReceiveProtoT data
     * @param ip_address The ip address to listen on for
     * the given ReceiveProtoT packets (IPv4 in dotted decimal or IPv6 in hex string)
     *  example IPv4: 192.168.0.2
     *  example IPv6: ff02::c3d0:42d2:bb8%wlp4s0 (the interface is specified after %)
     * @param port The port on which to listen for ReceiveProtoT packets
     * @param receive_callback The function to run for every ReceiveProtoT packet received
     * from the network
     * @param multicast If true, joins the multicast group of given ip_address
     */
    ProtoUdpListener(boost::asio::io_service& io_service, const std::string& ip_address,
                     unsigned short port,
                     std::function<void(ReceiveProtoT&)> receive_callback,
                     bool multicast);

    virtual ~ProtoUdpListener();

   private:
    /**
     * This function is setup as the callback to handle packets received over the network.
     *
     * @param error The error code obtained when receiving the incoming data
     * @param num_bytes_received How many bytes of data were received
     */
    void handleDataReception(const boost::system::error_code& error,
                             size_t num_bytes_received);

    // A UDP socket that we listen on for ReceiveProtoT messages from the network
    boost::asio::ip::udp::socket socket_;
    // The endpoint for the sender
    boost::asio::ip::udp::endpoint sender_endpoint_;

    static constexpr unsigned int MAX_BUFFER_LENGTH = 9000;
    std::array<char, MAX_BUFFER_LENGTH> raw_received_data_;

    // The function to call on every received packet of ReceiveProtoT data
    std::function<void(ReceiveProtoT&)> receive_callback;
};

#include "software/networking/proto_udp_listener.tpp"
