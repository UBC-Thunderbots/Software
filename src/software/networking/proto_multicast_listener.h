#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>

template <class ReceiveProto>
class ProtoMulticastListener
{
   public:
    /**
     * Creates an ProtoMulticastListener that will listen for ReceiveProto packets from
     * the network on the given address and port. For every ReceiveProto packet received,
     * the receive_callback will be called to perform any operations desired by the caller
     *
     * @param io_service The io_service to use to service incoming ReceiveProto data
     * @param ip_address The ip address of the multicast group on which to listen for
     * the given ReceiveProto packets (IPv4 in dotted decimal or IPv6 in hex string)
     * @param port The port on which to listen for ReceiveProto packets
     * @param receive_callback The function to run for every ReceiveProto packet received
     * from the network
     */
    ProtoMulticastListener(boost::asio::io_service& io_service,
                           const std::string& ip_address, unsigned short port,
                           std::function<void(ReceiveProto)> receive_callback);

    virtual ~ProtoMulticastListener();

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
    std::function<void(ReceiveProto)> receive_callback;
};

#include "software/networking/proto_multicast_listener.tpp"
