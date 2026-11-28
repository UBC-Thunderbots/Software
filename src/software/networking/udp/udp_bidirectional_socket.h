#pragma once

#include <boost/asio.hpp>
#include <memory>
#include <string>

#include "software/networking/udp/udp_listener.h"

/**
 * A UdpListener that can also send from its socket.
 *
 * Constructors are inherited from UdpListener.
 */
class UdpBidirectionalSocket : public UdpListener
{
   public:
    using UdpListener::UdpListener;

    /**
     * Sends a datagram through the listening socket.
     *
     * This posts to the `io_context`, so that needs to run for the send
     * to go through.
     *
     * @param message The bytes to send
     * @param destination The recipient's address and port
     */
    void sendTo(const std::string& message,
                const boost::asio::ip::udp::endpoint& destination);

   private:
    /**
     * The sending logic that runs within the `io_context` thread.
     */
    void doSend(std::shared_ptr<std::string> buf,
                const boost::asio::ip::udp::endpoint& destination);
};
