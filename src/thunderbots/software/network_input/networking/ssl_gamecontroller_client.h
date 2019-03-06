#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>

#include "proto/ssl_referee.pb.h"

class SSLGameControllerClient
{
   public:
    /**
     * Creates an SSLGameControllerClient that will listen for data packets from the SSL
     * Game Controller on the given address and port. For every controller packet
     * received, the handle_function will be cald to perform any operations desired by the
     * caller
     *
     * @param io_service The io_service to use to service incoming GameController data
     * @param ip_address The ip address of the multicast group on which to listen for
     * GameController packets
     * @param port The port on which to listen for GameController packets
     * @param handle_function The function to run for every GameController packet received
     * from the network
     */
    SSLGameControllerClient(boost::asio::io_service& io_service, std::string ip_address,
                            unsigned short port,
                            std::function<void(Referee)> handle_function);

   private:
    /**
     * The function that is called to process any data received by the io_service. This
     * function gets automatically called by the io_service for EVERY packet received by
     * the network. These functions are handled synchronously, so we DO NOT need to worry
     * about concurrency or thread-safety in this function. Because this function also
     * calls the provided handle_function, this means the handle_function also does not
     * need to be thread-safe
     *
     * @param error The error code obtained when receiving the incoming data
     * @param num_bytes_received How many bytes of data were received
     */
    void handleDataReception(const boost::system::error_code& error,
                             size_t num_bytes_received);

    // A UDP socket that we listen on for protobuf messages from SSL Game Controller
    boost::asio::ip::udp::socket socket_;
    // The endpoint for the sender, in this case SSL Game Controller
    boost::asio::ip::udp::endpoint sender_endpoint_;

    // The maximum length of the buffer we use to receive data packets from the network
    static constexpr unsigned int max_buffer_length = 4096;
    // Acts as a buffer to store the raw received data from the network
    std::array<char, max_buffer_length> raw_received_data_;
    // The function to call on every received packet of vision data
    std::function<void(Referee)> handle_function;
};
