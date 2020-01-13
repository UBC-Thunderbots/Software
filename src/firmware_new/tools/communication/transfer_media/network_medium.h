#pragma once
#include "boost/array.hpp"
#include "boost/asio.hpp"
#include "boost/bind.hpp"
#include "firmware_new/tools/communication/transfer_media/transfer_medium.h"
#include "software/multithreading/thread_safe_buffer.h"

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
    NetworkMedium() = delete;
    virtual ~NetworkMedium();

    NetworkMedium(std::string local_ipaddr, unsigned port);
    /*
     * Packages the data and sends it through the medium
     *
     * @param data The data to be sent over the medium
     */
    void send_data(const std::string& data);

    /*
     * Returns when a data packet has arrived through the medium
     *
     * NOTE: Depending on the implementation of send_data
     * this function may be triggered by a packet sent by the
     * same instance of the class. (example: UDP broadcast, the sender
     * also gets a packet)
     *
     * @param func The function to call when data is received
     */
    void receive_data(std::function<void(std::string)> receive_callback);

   private:
    std::string data_buffer;
    std::unique_ptr<udp::socket> socket;
    boost::asio::io_service io_service;
    udp::endpoint local_endpoint;
    udp::endpoint remote_endpoint;
};
