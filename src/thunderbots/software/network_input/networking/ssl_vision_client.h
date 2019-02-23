#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <queue>
#include <string>

#include "proto/messages_robocup_ssl_wrapper.pb.h"

class SSLVisionClient
{
   public:
    /**
     * Creates an SSLVisionClient that will listen for data packets from SSL Vision
     * on the given address and port
     *
     * @param ip_address The ip address of the multicast group on which to listen for
     * SSL Vision packets
     * @param port The port on which to listen for SSL Vision packets
     */
    SSLVisionClient(const std::string ip_address, const unsigned short port);

    /**
     * Returns a vector of new SSL Vision WrapperPackets that were received since last
     * time this function was called. If no new data was received since the last time this
     * function was called, the vector will be empty
     *
     * @return A vector of SSL_WrapperPackets received since the last time this function
     * was called.
     */
    const std::vector<SSL_WrapperPacket> getVisionPacketVector();

   private:
    /**
     * The function that is called to process any data received by the io_service.
     * When io_service.poll() is called, this function will be run for EVERY packet
     * received, which may be more than once.
     *
     * @param error The error code obtained when receiving the incoming data
     * @param num_bytes_received How many bytes of data were received
     */
    void handleDataReception(const boost::system::error_code &error,
                             size_t num_bytes_received);

    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint sender_endpoint_;

    // The maximum length of the buffer we use to receive data packets from the network
    static constexpr unsigned int max_buffer_length = 4096;
    // Acts as a buffer to store the raw received data from the network
    std::array<char, max_buffer_length> raw_received_data_;
    // Stores the SSL_WrapperPackets that were received
    std::vector<SSL_WrapperPacket> packet_vector;
};
