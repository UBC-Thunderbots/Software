#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

#include "software/logger/logger.h"
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
     * @throws TbotsNetworkException if the multicast group could not be joined if the
     * multicast option is requested
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
     */
    ProtoUdpListener(boost::asio::io_service& io_service, const std::string& ip_address,
                     unsigned short port, const std::string& listen_interface,
                     std::function<void(ReceiveProtoT&)> receive_callback,
                     bool multicast);

    /**
     * Creates an ProtoUdpListener that will listen for ReceiveProtoT packets from
     * the network on any local address with given port. For every ReceiveProtoT packet
     * received, the receive_callback will be called to perform any operations desired by
     * the caller
     *
     * @throws TbotsNetworkException if the multicast group could not be joined if the
     * multicast option is requested
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

    /**
     * Destructor
     */
    virtual ~ProtoUdpListener();

   private:
    /**
     * This function is setup as the callback to handle packets received over the network.
     *
     * @param buffer A buffer containing the raw data received
     * @param num_bytes_received The number of bytes received that is valid in the buffer
     */
    void handleDataReception(const char* buffer, const size_t& num_bytes_received);

    /**
     * Start listening for data
     */
    void startListen();

    UdpListener udp_listener_;  // The underlying UDP listener
    // The function to call on every received packet of ReceiveProtoT data
    std::function<void(ReceiveProtoT&)> receive_callback;
};

template <class ReceiveProtoT>
ProtoUdpListener<ReceiveProtoT>::ProtoUdpListener(
    boost::asio::io_service& io_service, const std::string& ip_address,
    const unsigned short port, const std::string& listen_interface,
    std::function<void(ReceiveProtoT&)> receive_callback, bool multicast)
    : udp_listener_(io_service, ip_address, port, listen_interface, multicast,
                    std::bind(handleDataReception, this, std::placeholders::_1)),
      receive_callback(receive_callback)
{
}

template <class ReceiveProtoT>
ProtoUdpListener<ReceiveProtoT>::ProtoUdpListener(
    boost::asio::io_service& io_service, const unsigned short port,
    std::function<void(ReceiveProtoT&)> receive_callback,
    std::optional<std::string>& error)
    : udp_listener_(io_service, port,
                    std::bind(handleDataReception, this, std::placeholders::_1)),
      receive_callback(receive_callback)
{
}

template <class ReceiveProtoT>
void ProtoUdpListener<ReceiveProtoT>::handleDataReception(
    const char* buffer, const size_t& num_bytes_received)
{
    auto packet_data = ReceiveProtoT();
    packet_data.ParseFromArray(raw_received_data_.data(),
                               static_cast<int>(num_bytes_received));
    receive_callback(packet_data);
}

template <class ReceiveProtoT>
ProtoUdpListener<ReceiveProtoT>::~ProtoUdpListener()
{
}

template <class ReceiveProtoT>
void ProtoUdpListener<ReceiveProtoT>::close()
{
    udp_listener_.close();
}
