#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>

#include "software/logger/logger.h"
#include "software/networking/proto_udp_listener.hpp"
#include "software/networking/udp_listener.h"
#include "software/util/typename/typename.h"

template <class ReceiveProtoT>
class ProtoUdpListener
{
   public:
    /**
     * Creates an ProtoUdpListener that will listen for ReceiveProtoT packets from
     * the network on the multicast group of given address and port. For every
     * ReceiveProtoT packet received, the receive_callback will be called to perform any
     * operations desired by the caller
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
    ProtoUdpListener(boost::asio::io_service& io_service, const std::string& ip_address,
                     unsigned short port,
                     std::function<void(ReceiveProtoT&)> receive_callback,
                     bool multicast);

    /**
     * Creates an ProtoUdpListener that will listen for ReceiveProtoT packets from
     * the network on any local address with given port. For every ReceiveProtoT packet
     * received, the receive_callback will be called to perform any operations desired by
     * the caller
     *
     * @param io_service The io_service to use to service incoming ReceiveProtoT data
     * @param port The port on which to listen for ReceiveProtoT packets
     * @param receive_callback The function to run for every ReceiveProtoT packet received
     * from the network
     */
    ProtoUdpListener(boost::asio::io_service& io_service, unsigned short port,
                     std::function<void(ReceiveProtoT&)> receive_callback);
    /**
     * Closes the socket associated to the UDP listener
     */
    virtual void close();

    virtual ~ProtoUdpListener();


   private:
    void handleRawData(const char* raw_data, const size_t& num_bytes_received);

    UdpListener udp_listener_;
};

template <class ReceiveProtoT>
ProtoUdpListener<ReceiveProtoT>::ProtoUdpListener(
    boost::asio::io_service& io_service, const std::string& ip_address,
    const unsigned short port, std::function<void(ReceiveProtoT&)> receive_callback,
    bool multicast)
    : udp_listener_(io_service, ip_address, port, multicast, &ProtoUdpListener::handleRawData)
{
}

template <class ReceiveProtoT>
ProtoUdpListener<ReceiveProtoT>::ProtoUdpListener(
    boost::asio::io_service& io_service, const unsigned short port,
    std::function<void(ReceiveProtoT&)> receive_callback)
    : udp_listener_(io_service, port, &ProtoUdpListener::handleRawData)
{
}

template <class ReceiveProtoT>
void ProtoUdpListener<ReceiveProtoT>::handleRawData(
    const char* raw_data, const size_t& num_bytes_received)
{
    auto packet_data = ReceiveProtoT();
    packet_data.ParseFromArray(raw_data,
                               static_cast<int>(num_bytes_received));
    receive_callback(packet_data);
}

template <class ReceiveProtoT>
ProtoUdpListener<ReceiveProtoT>::~ProtoUdpListener()
{
    delete udp_listener_;
}

template <class ReceiveProtoT>
void ProtoUdpListener<ReceiveProtoT>::close()
{
    udp_listener_.close();
}
