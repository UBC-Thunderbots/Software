#include "network_input/networking/ssl_vision_client.h"

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
        // TODO (Issue #33): Add a log message with something like the
        // following here:
        // "there was an issue binding the socket to the endpoint when trying to
        // connect to the SSL vision server. This may be due to another instance
        // of network_input running and using the port already"

        // Throw this exception up to top-level, as we have no valid
        // recovery action here
        throw;
    }

    // Join the multicast group.
    socket_.set_option(boost::asio::ip::multicast::join_group(
        boost::asio::ip::address::from_string(ip_address)));

    // Start listening for data
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
        // We create a new pointer here rather than reset or clear the existing pointer
        // because the unique_ptr is returned with std::move in the getVisionPacket()
        // function, meaning we can no longer access that memory. Hence we create a new
        // pointer to work with
        packet_data = std::make_unique<SSL_WrapperPacket>();
        packet_data->ParseFromArray(raw_received_data_,
                                    static_cast<int>(num_bytes_received));

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

        // TODO: Log a proper warning here so we can be notified that there was an error
        // while handling received data. Can be done once
        // https://github.com/UBC-Thunderbots/Software/issues/33 is done
    }
}

const std::unique_ptr<SSL_WrapperPacket> SSLVisionClient::getVisionPacket()
{
    // Polling the service causes the handleDataReception function to run, storing
    // data in the packet_data variable, which we can then return
    io_service.poll();
    return std::move(packet_data);
}
