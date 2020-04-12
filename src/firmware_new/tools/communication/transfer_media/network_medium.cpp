#include "firmware_new/tools/communication/transfer_media/network_medium.h"

#include "software/logger/logger.h"

#include "boost/array.hpp"
#include "boost/asio.hpp"
#include "boost/asio/error.hpp"
#include "boost/bind.hpp"

using boost::asio::socket_base;
using boost::asio::ip::udp;

NetworkMedium::NetworkMedium(const std::string& multicast_address,
                             unsigned multicast_port, unsigned receive_port)
{
    send_socket.reset(new udp::socket(io_service));
    recv_socket.reset(new udp::socket(io_service));

    boost::asio::ip::address multicast_addr =
        boost::asio::ip::address_v6::from_string(multicast_address);

    local_endpoint     = udp::endpoint(boost::asio::ip::address_v6::any(), receive_port);
    multicast_endpoint = udp::endpoint(multicast_addr, multicast_port);

    recv_socket->open(local_endpoint.protocol());
    send_socket->open(multicast_endpoint.protocol());

    send_socket->set_option(boost::asio::ip::multicast::join_group(multicast_addr));
    send_socket->set_option(boost::asio::ip::multicast::enable_loopback(false));

    try
    {
        send_socket->bind(multicast_endpoint);
        recv_socket->bind(local_endpoint);
    }
    catch (const boost::exception& ex)
    {
        LOG(WARNING) << "There was an issue binding the socket to the endpoint when"
                        "trying to connect to the provided port"
                        "Please make sure no other program is using the port"
                     << std::endl;

        // Throw this exception up to top-level, as we have no valid
        // recovery action here
        throw;
    }
}

NetworkMedium::~NetworkMedium()
{
    // Stop the io_service. This is safe to call from another thread.
    // https://stackoverflow.com/questions/4808848/boost-asio-stopping-io-service
    // This MUST be done before attempting to join the thread because otherwise the
    // io_service will not stop and the thread will not join
    io_service.stop();

    // Join the io_service_thread so that we wait for it to exit before destructing the
    // thread object. If we do not wait for the thread to finish executing, it will call
    // `std::terminate` when we deallocate the thread object and kill our whole program
    io_service_thread.join();

    recv_socket->close();
    recv_socket.reset();
    send_socket->close();
    send_socket.reset();
}

void NetworkMedium::send_data(const std::string& data)
{
    send_socket->send_to(boost::asio::buffer(data), multicast_endpoint);
}

void NetworkMedium::receive_data_async(std::function<void(std::string)> receive_callback)
{
    this->receive_callback = receive_callback;

    recv_socket->async_receive_from(
        boost::asio::buffer(data_buffer, max_buffer_length), local_endpoint,
        boost::bind(&NetworkMedium::handle_data_reception, this,
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));

    // start the thread to run the io_service in the background
    io_service_thread = std::thread([this]() { io_service.run(); });
}

void NetworkMedium::handle_data_reception(const boost::system::error_code& error,
                                          size_t num_bytes_received)
{
    if (!error)
    {
        receive_callback(std::string(data_buffer.begin(), data_buffer.end()));
    }
    else
    {
        LOG(WARNING) << "An unknown network error occurred when attempting"
                        "to receive the packet. The boost system error code is "
                     << error << std::endl;
    }

    recv_socket->async_receive_from(
        boost::asio::buffer(data_buffer, max_buffer_length), local_endpoint,
        boost::bind(&NetworkMedium::handle_data_reception, this,
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
}
