#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>

#include "software/logger/logger.h"
#include "software/networking/udp_listener.h"
#include "software/networking/udp/network_utils.h"
#include "software/networking/udp/proto_udp_listener.hpp"
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
     * The caller must check that the error is not set before using the listener
     *
     * @param io_service The io_service to use to service incoming ReceiveProtoT data
     * @param ip_address The ip address of on which to listen for the given ReceiveProtoT
     * packets (IPv4 in dotted decimal or IPv6 in hex string) example IPv4: 192.168.0.2
     *  example IPv6: ff02::c3d0:42d2:bb8
     * @param port The port on which to listen for ReceiveProtoT packets
     * @param listen_interface The interface to listen on
     * @param receive_callback The function to run for every ReceiveProtoT packet received
     * from the network
     * @param multicast If true, joins the multicast group of given ip_address
     * @param error A user-provided optional string to store any error messages
     */
    ProtoUdpListener(boost::asio::io_service& io_service, const std::string& ip_address,
                     unsigned short port, const std::string& listen_interface,
                     std::function<void(ReceiveProtoT&)> receive_callback, bool multicast,
                     std::optional<std::string>& error);

    /**
     * Creates an ProtoUdpListener that will listen for ReceiveProtoT packets from
     * the network on any local address with given port. For every ReceiveProtoT packet
     * received, the receive_callback will be called to perform any operations desired by
     * the caller
     *
     * The caller must check that the error is not set before using the listener
     *
     * @param io_service The io_service to use to service incoming ReceiveProtoT data
     * @param port The port on which to listen for ReceiveProtoT packets
     * @param receive_callback The function to run for every ReceiveProtoT packet received
     * from the network
     * @param error A user-provided optional string to store any error messages
     */
    ProtoUdpListener(boost::asio::io_service& io_service, unsigned short port,
                     std::function<void(ReceiveProtoT&)> receive_callback,
                     std::optional<std::string>& error);
    /**
     * Closes the socket associated to the UDP listener
     */
    virtual void close();

    virtual ~ProtoUdpListener();


   private:
    void handleRawData(const char* raw_data, const size_t& num_bytes_received);

    UdpListener udp_listener_;

    // The function to call on every received packet of ReceiveProtoT data
    std::function<void(ReceiveProtoT&)> receive_callback;
};

template <class ReceiveProtoT>
ProtoUdpListener<ReceiveProtoT>::ProtoUdpListener(
    boost::asio::io_service& io_service, const std::string& ip_address,
    const unsigned short port, const std::string& interface,
    std::function<void(ReceiveProtoT&)> receive_callback,
    bool multicast, std::optional<std::string>& error)
    : udp_listener_(io_service, ip_address, port, multicast,
                    &ProtoUdpListener::handleRawData, error),
      receive_callback(receive_callback)
{
}

template <class ReceiveProtoT>
ProtoUdpListener<ReceiveProtoT>::ProtoUdpListener(
    boost::asio::io_service& io_service,
    const unsigned short port, const std::string& interface,
    std::function<void(ReceiveProtoT&)> receive_callback,
    std::optional<std::string>& error)
    : udp_listener_(io_service, port, &ProtoUdpListener::handleRawData),
      receive_callback(receive_callback)
{
}

template <class ReceiveProtoT>
void ProtoUdpListener<ReceiveProtoT>::handleRawData(const char* raw_data,
                                                    const size_t& num_bytes_received)
{
    auto packet_data = ReceiveProtoT();
    packet_data.ParseFromArray(raw_data, static_cast<int>(num_bytes_received));
    receive_callback(packet_data);
}

template <class ReceiveProtoT>
void ProtoUdpListener<ReceiveProtoT>::close()
{
    udp_listener_.close();
}
