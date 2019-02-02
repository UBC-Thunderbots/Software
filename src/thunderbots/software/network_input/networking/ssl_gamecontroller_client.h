#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>

#include "proto/ssl_referee.pb.h"

class SSLGameControllerClient
{
   public:
    /**
     * Creates an SSLGameControllerClient that will listen for data packets from SSL Game
     * Controller on the given address and port
     *
     * @param ip_address The ip address of the multicast group on which to listen for
     * SSL Vision packets
     * @param port The port on which to listen for SSL Game Controller packets
     */
    SSLGameControllerClient(std::string ip_address, unsigned short port);

    /**
     * Returns a unique_ptr to the latest SSL GameController Referee packet. If no new
     * data was received since the last time this function was called, the unique_ptr will
     * be empty
     *
     * @return A unique_ptr to an Referee packet if data was received since the last
     * time this function was called, otherwise returns an empty unique_ptr
     */
    const std::unique_ptr<Referee> getGameControllerPacket();

   private:
    /**
     * The function that is called to process any data received by the io_service.
     * This function will be called when poll() is called on the io_service.
     *
     * @param error The error code obtained when receiving the incoming data
     * @param num_bytes_received How many bytes of data were received
     */
    void handleDataReception(const boost::system::error_code &error,
                             size_t num_bytes_received);

    // provides core I/O functionality for boost asynchronous I/O objects such as a socket
    boost::asio::io_service io_service;

    // a UDP socket that we listen on for protobuf messages from SSL Game Controller
    boost::asio::ip::udp::socket socket_;

    // the endpoint for the sender, in this case SSL Game Controller
    boost::asio::ip::udp::endpoint sender_endpoint_;

    // We set the max buffer length to be the largest possible size of a UDP datagram.
    // This way we don't need to worry about buffer overflow, or about our data getting
    // truncated to fit into the buffer
    static constexpr unsigned int max_buffer_length = 65535;
    char raw_received_data_[max_buffer_length];

    // Stores the most up to date packet data received by the client
    std::unique_ptr<Referee> packet_data;
};
