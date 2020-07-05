#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>

class MulticastListener
{
   public:
    /**
     * Creates a MulticastListener that will listen for packets from the network on a
     * given address in port. For every packet received, the receive_callback will be
     * called to perform any operations desired by the caller
     *
     * @param io_service The io_service to use to service incoming packets data
     * @param ip_address The ip address of the multicast group on which to listen for
     * the packets (IPv4 in dotted decimal or IPv6 in hex string)
     *  example IPv4: 192.168.0.2
     *  example IPv6: ff02::c3d0:42d2:bb8%wlp4s0 (the interface is specified after %)
     * @param port The port on which to listen for packets
     * @param receive_callback The function to run for every packet received
     * from the network
     */
    MulticastListener(boost::asio::io_service& io_service, const std::string& ip_address,
                      unsigned short port,
                      std::function<void(std::vector<uint8_t>&)> receive_callback);

    virtual ~MulticastListener();

   private:
    /**
     * The function that is called to process any data received by the io_service. This
     * function gets automatically called by the io_service for EVERY packet received by
     * the network. These functions are handled synchronously, so we DO NOT need to worry
     * about concurrency or thread-safety in this function. Because this function also
     * calls the provided receive_callback, this means the receive_callback also does not
     * need to be thread-safe
     *
     * @param error The error code obtained when receiving the incoming data
     * @param num_bytes_received How many bytes of data were received
     */
    void handleDataReception(const boost::system::error_code& error,
                             size_t num_bytes_received);

    // A UDP socket that we listen on for ReceiveProto messages from the network
    boost::asio::ip::udp::socket socket_;
    // The endpoint for the sender
    boost::asio::ip::udp::endpoint sender_endpoint_;

    // The maximum length of the buffer we use to receive Protobuf packets from the
    // network. 9000 bytes is the maximum transfer unit (MTU) size we can set on our
    // network interfaces, before the frame will be fragmented when it is sent, because
    // that is the maximum size of a jumbo frame https://en.wikipedia.org/wiki/Jumbo_frame
    // Any proto message received by ProtoMulticastListener must be smaller than this
    // length in order to be processed correctly
    static constexpr unsigned int max_buffer_length = 9000;
    // Acts as a buffer to store the raw received data from the network
    std::array<char, max_buffer_length> raw_received_data_;
    // The function to call on every received packet of ReceiveProto data
    std::function<void(std::vector<uint8_t>&)> receive_callback;
};
