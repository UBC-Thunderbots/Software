#include "network_input/networking/ssl_gamecontroller_client.h"

#include "util/logger/init.h"

SSLGameControllerClient::SSLGameControllerClient(const std::string ip_address,
                                                 const unsigned short port)
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
        LOG(INFO) << "SSL Game Controller client could not bind to socket!";
        throw;
    }

    // Join the multicast group.
    socket_.set_option(boost::asio::ip::multicast::join_group(
        boost::asio::ip::address::from_string(ip_address)));

    // Start listening for data
    socket_.async_receive_from(boost::asio::buffer(raw_received_data_, max_buffer_length),
                               sender_endpoint_,
                               boost::bind(&SSLGameControllerClient::handleDataReception,
                                           this, boost::asio::placeholders::error,
                                           boost::asio::placeholders::bytes_transferred));
}

void SSLGameControllerClient::handleDataReception(const boost::system::error_code& error,
                                                  size_t num_bytes_received)
{
    if (!error)
    {
        // We create a new pointer here rather than reset or clear the existing pointer
        // because the unique_ptr is returned with std::move in the getVisionPacket()
        // function, meaning we can no longer access that memory. Hence we create a new
        // pointer to work with
        packet_data = std::make_unique<Referee>();
        packet_data->ParseFromArray(raw_received_data_,
                                    static_cast<int>(num_bytes_received));

        // Once we've handled the data, start listening again
        socket_.async_receive_from(
            boost::asio::buffer(raw_received_data_, max_buffer_length), sender_endpoint_,
            boost::bind(&SSLGameControllerClient::handleDataReception, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));
    }
    else
    {
        // Start listening again to receive the next data
        socket_.async_receive_from(
            boost::asio::buffer(raw_received_data_, max_buffer_length), sender_endpoint_,
            boost::bind(&SSLGameControllerClient::handleDataReception, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));

        LOG(WARNING) << "Error while receiving data from SSL Game Controller: " << error;
    }
}

const std::unique_ptr<Referee> SSLGameControllerClient::getGameControllerPacket()
{
    // Polling the service causes the handleDataReception function to run, storing
    // data in the packet_data variable, which we can then return
    io_service.poll();
    return std::move(packet_data);
}
