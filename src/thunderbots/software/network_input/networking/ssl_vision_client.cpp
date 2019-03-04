#include "network_input/networking/ssl_vision_client.h"

#include "util/logger/init.h"

SSLVisionClient::SSLVisionClient(boost::asio::io_service& io_service,
                                 const std::string ip_address, const unsigned short port,
                                 std::function<void(SSL_WrapperPacket)> handle_function)
    : socket_(io_service), handle_function(handle_function)
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
        auto packet_data = SSL_WrapperPacket();
        packet_data.ParseFromArray(raw_received_data_.data(),
                                   static_cast<int>(num_bytes_received));
        handle_function(packet_data);

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
