#pragma once

#include <functional>
#include <string>
#include <thread>
#include <utility>

#include "software/networking/udp/proto_udp_bidirectional.hpp"

/**
 * A threaded endpoint that receives serialized ReceiveProtoT Proto's over the network and
 * can send serialized SendProtoT Proto's back
 */
template <class ReceiveProtoT, class SendProtoT>
class ThreadedProtoUdpBidirectional
{
   public:
    using Callback =
        std::function<void(ReceiveProtoT&, const boost::asio::ip::udp::endpoint&)>;

    /**
     * Creates a ThreadedProtoUdpBidirectional that will listen for ReceiveProtoT packets
     * from the network of given address and port. For every ReceiveProtoT packet
     * received, the callback will be called on the IO thread to perform any operations
     * desired by the caller. The ThreadedProtoUdpBidirectional can also send packets of
     * type SendProtoT.
     *
     * @throws TbotsNetworkException if we detect an issue with setting up this endpoint
     *
     * @param address The ip address on which to listen for the given ReceiveProtoT
     * packets (IPv4 in dotted decimal or IPv6 in hex string) example IPv4: 192.168.0.2
     *  example IPv6: ff02::c3d0:42d2:bb8%wlp4s0
     * @param port The port on which to listen for ReceiveProtoT packets
     * @param interface The interface on which to listen for ReceiveProtoT packets
     * @param callback The function to run for every ReceiveProtoT packet received from
     * the network, along with the endpoint that sent it
     * @param multicast If true, joins the multicast group of given address
     */
    ThreadedProtoUdpBidirectional(const std::string& address, unsigned short port,
                                  const std::string& interface, Callback callback,
                                  bool multicast)
        : udp(io_service, address, port, interface, std::move(callback), multicast),
          io_thread([this]() { io_service.run(); })
    {
    }

    /**
     * Creates a ThreadedProtoUdpBidirectional that will listen for ReceiveProtoT packets
     * from the network on any local address with given port. For every ReceiveProtoT
     * packet received, the callback will be called on the IO thread to perform any
     * operations desired by the caller. The ThreadedProtoUdpBidirectional can also send
     * packets of type SendProtoT. This constructor should not be used for multicast
     * communication.
     *
     * @throws TbotsNetworkException if we detect an issue with setting up this endpoint
     *
     * @param port The port on which to listen for ReceiveProtoT packets
     * @param callback The function to run for every ReceiveProtoT packet received from
     * the network, along with the endpoint that sent it
     */
    ThreadedProtoUdpBidirectional(unsigned short port, Callback callback)
        : udp(io_service, port, std::move(callback)),
          io_thread([this]() { io_service.run(); })
    {
    }

    /**
     * Destructor will close the socket and stop the IO thread
     */
    ~ThreadedProtoUdpBidirectional()
    {
        close();
    }

    /**
     * Sends the message to the specified endpoint. The send happens asynchronously on the
     * IO thread, so this is safe to call from the receive callback or any other thread.
     * Sends that have not gone out yet are dropped when the endpoint is closed.
     *
     * @param message The proto message to send
     * @param destination The endpoint to send the message to (likely the sender endpoint
     * from the callback)
     */
    void sendTo(const SendProtoT& message,
                const boost::asio::ip::udp::endpoint& destination)
    {
        udp.sendTo(message, destination);
    }

    /**
     * Stops and joins the IO thread, then closes the socket. Do not call from the
     * receive callback, and don't call when holding a lock that the receive callback
     * needs.
     */
    void close()
    {
        if (io_thread.joinable())
        {
            io_service.stop();
            io_thread.join();
            udp.close();
        }
    }

   private:
    boost::asio::io_service io_service;
    ProtoUdpBidirectional<ReceiveProtoT, SendProtoT> udp;
    std::thread io_thread;
};
