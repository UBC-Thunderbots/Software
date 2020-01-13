#include "firmware_new/tools/communication/transfer_media/network_medium.h"

#include "boost/array.hpp"
#include "boost/asio.hpp"
#include "boost/asio/error.hpp"
#include "boost/bind.hpp"

using boost::asio::socket_base;
using boost::asio::ip::address_v4;
using boost::asio::ip::udp;

NetworkMedium::NetworkMedium(std::string local_ipaddr, unsigned port)
{
    socket.reset(new udp::socket(io_service));

    // TODO explain endpoints
    local_endpoint  = udp::endpoint(address_v4::from_string(local_ipaddr), port);
    remote_endpoint = udp::endpoint(address_v4::broadcast(), port);

    boost::system::error_code error;
    socket->open(udp::v4(), error);

    if (!error)
    {
        // TODO explain options
        socket->set_option(udp::socket::reuse_address(true));
        socket->set_option(socket_base::broadcast(true));
    }

    socket->bind(local_endpoint);
    io_service.run();
}

NetworkMedium::~NetworkMedium()
{
    socket->close();
    socket.reset();
}

void NetworkMedium::send_data(const std::string& data)
{
    socket->send_to(boost::asio::buffer(data), remote_endpoint);
}

void handle_receive_from(const boost::system::error_code& error, size_t bytes_recvd) {}

void NetworkMedium::receive_data(std::function<void(std::string)> receive_callback)
{
    // Implemented according to this to continue receiving data
    // https://www.boost.org/doc/libs/1_66_0/doc/html/boost_asio/example/cpp11/echo/async_udp_echo_server.cpp

    // TODO fix 1000 fixed syse
    std::vector<char> buffer(1000);

    if (!error && bytes_recvd > 0)
    {
        received_callback(std::string(buffer.begin(), buffer.end()));
    }
    else
    {
        socket->async_receive_from(
            boost::asio::buffer(data_, max_length), sender_endpoint_,
            boost::bind(&NetworkMedium::receive_data, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));
    }

    io_service.run();
}
