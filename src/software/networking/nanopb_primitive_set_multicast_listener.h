#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>

#include "software/networking/multicast_listener.h"
#include "software/world/robot_state.h"

extern "C" {
#include "shared/proto/tbots_software_msgs.nanopb.h"
}

class NanoPbPrimitiveSetMulticastListener
{
   public:
    /**
     * Creates an NanoPbPrimitiveSetMulticastListener that will listen for primitive sets
     * sent over the network on the given address and port. For every ReceiveProtoT packet
     * received, the receive_callback will be called to perform any operations desired by
     * the caller
     *
     * @param io_service The io_service to use to service incoming ReceiveProtoT data
     * @param ip_address The ip address of the multicast group on which to listen for
     * the given ReceiveProtoT packets (IPv4 in dotted decimal or IPv6 in hex string)
     *  example IPv4: 192.168.0.2
     *  example IPv6: ff02::c3d0:42d2:bb8%wlp4s0 (the interface is specified after %)
     * @param port The port on which to listen for ReceiveProtoT packets
     * @param receive_callback This function will be called with a map of
     *                         RobotId -> PrimitiveMsg
     */
    NanoPbPrimitiveSetMulticastListener(
        boost::asio::io_service& io_service, const std::string& ip_address,
        unsigned short port, std::function<void(PrimitiveSetMsg&)> receive_callback);

   private:
    /**
     * This function is setup as the callback to handle packets received over the network.
     *
     * @param data The packet received over the network
     */
    void handleDataReception(std::vector<uint8_t>& data);

    // The multicast listener that receives the data and passes it to us via
    // `handleDataReception`
    MulticastListener multicast_listener;

    // The function to call on every received packet of ReceiveProtoT data
    std::function<void(PrimitiveSetMsg&)> receive_callback;
};

