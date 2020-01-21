#pragma once
#include <thread>

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

    /*
     * Creates a NetworkMedium provided the ip and port
     *
     * @param local_ipaddr The local ip for this endpoint
     * @param port The port to bind to
     */
    NetworkMedium(const std::string& local_ipaddr, unsigned port);

    /*
     * Packages the data and sends it through the medium
     *
     * @param data The data to be sent over the medium
     */
    void send_data(const std::string& data);

    /*
     * Registers the callback to call when new data is received
     * This function is non-blocking, and returns immediately.
     *
     * @param func The function to call when data is received
     */
    void receive_data_async(std::function<void(std::string)> receive_callback);

   private:
    // The maximum length of the buffer we use to receive data packets from the network
    static constexpr unsigned int max_buffer_length = 4096;

    // Acts as a buffer to store the raw received data from the network
    std::array<char, max_buffer_length> data_buffer;

    // callback to run when new data arrives
    void handle_data_reception(const boost::system::error_code& error,
                               size_t num_bytes_received);

    // callback to run on recieve
    std::function<void(std::string)> receive_callback;

    // io_service to handle async calls
    boost::asio::io_service io_service;

    // local endpoint w/ given ip and given port
    udp::endpoint local_endpoint;

    // remote endpoint w/ broadcast ip and given port
    udp::endpoint broadcast_endpoint;

    // socket to send/recv
    std::unique_ptr<udp::socket> socket;

    // thread that runs all io_service related operations
    std::thread io_service_thread;
};
