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
     * the network on the multicast group of given address and port.
     *
     * @param io_service The io_service to use to service incoming ReceiveProtoT data
     * @param ip_address The ip address of on which to listen for the given ReceiveProtoT
     * packets (IPv4 in dotted decimal or IPv6 in hex string) example IPv4: 192.168.0.2
     *  example IPv6: ff02::c3d0:42d2:bb8%wlp4s0 (the interface is specified after %)
     * @param port The port on which to listen for ReceiveProtoT packets
     * @param multicast If true, joins the multicast group of given ip_address
     */
    ProtoUdpListener(boost::asio::io_service& io_service, const std::string& ip_address,
                     unsigned short port, bool multicast);

    /**
     * Creates an ProtoUdpListener that will listen for ReceiveProtoT packets from
     * the network on any local address with given port.
     *
     * @param io_service The io_service to use to service incoming ReceiveProtoT data
     * @param port The port on which to listen for ReceiveProtoT packets
     */
    ProtoUdpListener(boost::asio::io_service& io_service, unsigned short port);

    virtual ~ProtoUdpListener();

    /**
     * Blocks until a ReceiveProtoT has been received
     *
     * @returns ReceiveProtoT
     */
    ReceiveProtoT receiveProto();

   private:
    // A UDP socket that we listen on for ReceiveProtoT messages from the network
    boost::asio::ip::udp::socket socket_;

    // The endpoint for the sender
    boost::asio::ip::udp::endpoint sender_endpoint_;

    static constexpr unsigned int MAX_BUFFER_LENGTH = 9000;
    std::array<char, MAX_BUFFER_LENGTH> raw_received_data_;
};

#include "software/networking/proto_udp_listener.tpp"
