#pragma once
#include "boost/array.hpp"
#include "boost/asio.hpp"
#include "boost/bind.hpp"
#include "firmware_new/tools/communication/transfer_media/transfer_medium.h"

using boost::asio::ip::udp;

/*
 * NetworkMedium establishes a UDP socket and then broadcasts and listens
 * through that socket. There is no client/server model in-place, robots
 * and computers are all clients "peers" and broadcast msgs through the network.
 *
 * It is up to the peer to identify which msgs to process.
 *
 */

class NetworkMedium : public TransferMedium
{
   public:
    NetworkMedium();

   private:
    std::string data_buffer;
    std::unique_ptr<udp::socket> socket;
    boost::asio::io_service io_service;
    udp::endpoint local_endpoint;
    udp::endpoint remote_endpoint;
};
