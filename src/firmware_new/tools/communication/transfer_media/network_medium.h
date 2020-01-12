#pragma once
#include "boost/array.hpp"
#include "boost/asio.hpp"
#include "boost/bind.hpp"
#include "boost/error.hpp"
#include "firmware_new/tools/transfer_media/medium.h"

using boost::asio::ip::udp;

/*
 * NetworkMedium establishes a UDP socket and then broadcasts and listens
 * through that socket. There is no client/server model inplace, robots
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
    std::unique_ptr<udp::socket> send_socket;
    std::unique_ptr<udp::socket> receive_socket;
    std::unique_ptr<udp::endpoint> endpoint;
    std::unique_ptr<boost::asio::io_service> io_service;
};
