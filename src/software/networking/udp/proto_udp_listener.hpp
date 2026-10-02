#pragma once

#include <boost/asio.hpp>
#include <functional>
#include <string>
#include <utility>

#include "software/logger/logger.h"
#include "software/networking/udp/network_utils.h"
#include "software/networking/udp/udp_listener.h"
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
     * @param io_context The io_context to use to service incoming ReceiveProtoT data
     * @param ip_address The ip address of on which to listen for the given ReceiveProtoT
     * packets (IPv4 in dotted decimal or IPv6 in hex string) example IPv4: 192.168.0.2
     *  example IPv6: ff02::c3d0:42d2:bb8
     * @param port The port on which to listen for ReceiveProtoT packets
     * @param listen_interface The interface to listen on
     * @param receive_callback The function to run for every ReceiveProtoT packet received
     * from the network
     * @param multicast If true, joins the multicast group of given ip_address
     */
    ProtoUdpListener(boost::asio::io_context& io_context, const std::string& ip_address,
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
     * @param io_context The io_context to use to service incoming ReceiveProtoT data
     * @param port The port on which to listen for ReceiveProtoT packets
     * @param receive_callback The function to run for every ReceiveProtoT packet received
     * from the network
     */
    ProtoUdpListener(boost::asio::io_context& io_context, unsigned short port,
                     std::function<void(ReceiveProtoT&)> receive_callback);

    /**
     * Closes the socket associated to the UDP listener and blocks until the listener is
     * fully stopped. Must not be called from the io_context worker thread.
     */
    virtual void close();

    /**
     * Requests a stop of the underlying listener without blocking.
     */
    void requestStop();

    /**
     * Destructor
     */
    virtual ~ProtoUdpListener();

   private:
    UdpListener udp_listener_;  // The underlying UDP listener
};

template <class ReceiveProtoT>
ProtoUdpListener<ReceiveProtoT>::ProtoUdpListener(
    boost::asio::io_context& io_context, const std::string& ip_address,
    const unsigned short port, const std::string& listen_interface,
    std::function<void(ReceiveProtoT&)> receive_callback, bool multicast)
    : udp_listener_(io_context, ip_address, port, listen_interface, multicast,
                    [receive_callback = std::move(receive_callback)](
                        const char* buffer, const size_t& num_bytes_received)
                    {
                        auto packet_data = ReceiveProtoT();
                        std::ignore      = packet_data.ParseFromArray(
                                 buffer, static_cast<int>(num_bytes_received));
                        receive_callback(packet_data);
                    })
{
}

template <class ReceiveProtoT>
ProtoUdpListener<ReceiveProtoT>::ProtoUdpListener(
    boost::asio::io_context& io_context, const unsigned short port,
    std::function<void(ReceiveProtoT&)> receive_callback)
    : udp_listener_(io_context, port,
                    [receive_callback = std::move(receive_callback)](
                        const char* buffer, const size_t& num_bytes_received)
                    {
                        auto packet_data = ReceiveProtoT();
                        std::ignore      = packet_data.ParseFromArray(
                                 buffer, static_cast<int>(num_bytes_received));
                        receive_callback(packet_data);
                    })
{
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

template <class ReceiveProtoT>
void ProtoUdpListener<ReceiveProtoT>::requestStop()
{
    udp_listener_.requestStop();
}
