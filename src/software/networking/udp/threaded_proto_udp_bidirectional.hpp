#pragma once

#include <functional>
#include <string>
#include <thread>
#include <utility>

#include "software/networking/udp/proto_udp_bidirectional.hpp"

/** Runs protobuf UDP request/reply handling on an owned IO thread. */
template <class ReceiveProtoT, class SendProtoT>
class ThreadedProtoUdpBidirectional
{
   public:
    using Callback =
        std::function<void(ReceiveProtoT&, const boost::asio::ip::udp::endpoint&)>;

    ThreadedProtoUdpBidirectional(const std::string& address, unsigned short port,
                                  const std::string& interface, Callback callback,
                                  bool multicast)
        : udp(io_service, address, port, interface, std::move(callback), multicast),
          io_thread([this]() { io_service.run(); })
    {
    }

    ThreadedProtoUdpBidirectional(unsigned short port, Callback callback)
        : udp(io_service, port, std::move(callback)),
          io_thread([this]() { io_service.run(); })
    {
    }

    ~ThreadedProtoUdpBidirectional()
    {
        close();
    }

    /** Sends synchronously, including when called from the receive callback. */
    void sendTo(const SendProtoT& message,
                const boost::asio::ip::udp::endpoint& destination)
    {
        udp.sendTo(message, destination);
    }

    /** Stops and joins the IO thread. Do not call from the receive callback, and don't
     * call when holding a lock that the receive callback needs. */
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
