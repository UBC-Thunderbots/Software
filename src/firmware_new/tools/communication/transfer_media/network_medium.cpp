#include "firmware_new/tools/communication/transfer_media/network_medium.h"

#include "boost/array.hpp"
#include "boost/asio.hpp"
#include "boost/asio/error.hpp"
#include "boost/bind.hpp"

using boost::asio::socket_base;
using boost::asio::ip::address_v4;
using boost::asio::ip::udp;

void NetworkMedium::connect()
{
    // TODO explain options
    socket.reset(new udp::socket(io_service));
    socket->set_option(udp::socket::reuse_address(true));
    socket->set_option(socket_base::broadcast(true));

    // TODO explain endpoints
    local_endpoint  = udp::endpoint(address_v4::any(), 42069);
    remote_endpoint = udp::endpoint(address_v4::broadcast(), 42069);

    boost::system::error_code error;
    socket->open(udp::v4(), error);

    if (!error)
    {
        socket->set_option(udp::socket::reuse_address(true));
        socket->set_option(socket_base::broadcast(true));
    }

    try
    {
        socket->bind(local_endpoint);
    }
    catch (boost::system::system_error e)
    {
        // std::cerr << e.what() << std::endl;
    }
}

void NetworkMedium::disconnect()
{
    socket->close();
    socket.reset();
}

void NetworkMedium::send_data(const std::string& data)
{
    socket->send_to(boost::asio::buffer(data), remote_endpoint);
}

void NetworkMedium::receive_data(std::function<void(std::string)> receive_callback)
{
    // Implemented according to this to continue receiving data
    // https://www.boost.org/doc/libs/1_66_0/doc/html/boost_asio/example/cpp11/echo/async_udp_echo_server.cpp
    // socket->async_receive_from(
    // boost::asio::buffer(data_buffer, 1000), local_endpoint,
    //[this](boost::system::error_code ec, std::size_t bytes_recvd) {
    // if (!ec && bytes_recvd > 0)
    //{
    // receive_callback(data_buffer);
    //}
    // else
    //{
    // this->receive_data(receive_callback);
    //}
    //});
}
