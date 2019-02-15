#include "network_input/networking/ssl_vision_client.h"

#include "util/logger/init.h"

SSLVisionClient::SSLVisionClient(const std::string ip_address, const unsigned short port)
    : socket_(io_service)
{
    boost::asio::ip::udp::endpoint listen_endpoint(
        boost::asio::ip::address::from_string(ip_address), port);
    socket_.open(listen_endpoint.protocol());
    try
    {
        socket_.bind(listen_endpoint);
    }
    catch (const boost::exception& ex)
    {
        LOG(WARNING) << "There was an issue binding the socket to the endpoint when"
                        "trying to connect to the SSL Vision multicast address. This may"
                        "be due to another instance of the the SSLVisionClient running"
                        "and using the port already"
                     << std::endl;
        // Throw this exception up to top-level, as we have no valid
        // recovery action here
        throw;
    }

    // Join the multicast group.
    socket_.set_option(boost::asio::ip::multicast::join_group(
        boost::asio::ip::address::from_string(ip_address)));

    // Start listening for data asynchronously
    // See here for a great explanation about asynchronous operations:
    // https://stackoverflow.com/questions/34680985/what-is-the-difference-between-asynchronous-programming-and-multithreading
    socket_.async_receive_from(boost::asio::buffer(raw_received_data_, max_buffer_length),
                               sender_endpoint_,
                               boost::bind(&SSLVisionClient::handleDataReception, this,
                                           boost::asio::placeholders::error,
                                           boost::asio::placeholders::bytes_transferred));
}

void SSLVisionClient::handleDataReception(const boost::system::error_code& error,
                                          size_t num_bytes_received)
{
    if (!error)
    {
        // This function will be run for EVERY packet received from the network since the
        // last time poll() was called. This means that this function may be called
        // several times to process several packets. This is why we immediately store any
        // received and processed data into the packet_vector. If we didn't save the
        // received data somewhere else, the data would be overwritten by the packet
        // being handled in the next handleDataReception function call
        auto packet_data = SSL_WrapperPacket();
        packet_data.ParseFromArray(raw_received_data_.data(),
                                   static_cast<int>(num_bytes_received));
        packet_vector.emplace_back(packet_data);

        // Once we've handled the data, start listening again
        socket_.async_receive_from(
            boost::asio::buffer(raw_received_data_, max_buffer_length), sender_endpoint_,
            boost::bind(&SSLVisionClient::handleDataReception, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));
    }
    else
    {
        // Start listening again to receive the next data
        socket_.async_receive_from(
            boost::asio::buffer(raw_received_data_, max_buffer_length), sender_endpoint_,
            boost::bind(&SSLVisionClient::handleDataReception, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));

        LOG(WARNING)
            << "An unknown network error occurred when attempting to receive SSL Vision Data. The boost system error code is "
            << error << std::endl;
    }
}

const std::vector<SSL_WrapperPacket> SSLVisionClient::getVisionPacketVector()
{
    // Calling the poll() function will cause a handleDataReception() function to run for
    // EVERY packet of data that was received since the last time poll() was called.
    // Note that handleDataReception() may run multiple times in order to handle the
    // multiple received packets. This is why we move the data from the buffer into the
    // packet_vector, so that any subsequent handleDataReception() calls do not overwrite
    // the data
    io_service.poll();

    // Make a copy of the data to return
    std::vector<SSL_WrapperPacket> packet_vector_copy = packet_vector;
    // Clear the packet_vector
    packet_vector.clear();

    return packet_vector_copy;
}
