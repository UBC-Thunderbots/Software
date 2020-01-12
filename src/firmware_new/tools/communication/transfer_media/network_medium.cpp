#include "boost/array.hpp"
#include "boost/asio.hpp"
#include "boost/bind.hpp"
#include "boost/error.hpp"

using boost::asio::socket_base;
using boost::asio::ip::address_v4;
using boost::asio::ip::udp;

NetworkMedium::connect()
{
    boost::asio::error_code error;

    socket.reset(new udp::socket(io_service));
    socket->open(udp::v4(), error);

    if (!error)
    {
        socket.set_option(udp::socket::reuse_address(true));
        socket.set_option(socket_base::broadcast(true));
    }

    endpoint.reset(new address_v4::broadcast(), port);
    socket.open(ba::ip::udp::v4(), error);
}

NetworkMedium::disconnect()
{
    socket.close();
    socket.reset();
    endpoint.reset();
}

NetworkMedium::send_data(const std::string& data)
{
    boost::asio::error_code err;
    socket->send_to(boost::asio::buffer(data), endpoint, 0, err);
}

NetworkMedium::receive_data(std::function<void(std::string)> receive_callback)
{
    // Implemented according to this to continue receiving data
    // https://www.boost.org/doc/libs/1_66_0/doc/html/boost_asio/example/cpp11/echo/async_udp_echo_server.cpp
    socket->async_receive_from(
        boost::asio::buffer(data_buffer, max_length), enpoint,
        [this](boost::system::error_code ec, std::size_t bytes_recvd) {
            if (!ec && bytes_recvd > 0)
            {
                receive_callback(data);
            }
            else
            {
                this->receive_data(receive_callback);
            }
        });
}
