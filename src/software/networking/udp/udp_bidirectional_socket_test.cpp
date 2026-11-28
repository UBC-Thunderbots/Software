#include "software/networking/udp/udp_bidirectional_socket.h"

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

class UdpBidirectionalSocketTest : public testing::TestWithParam<bool>
{
};

TEST_P(UdpBidirectionalSocketTest, replies_to_saved_senders_from_the_bound_port)
{
    using Udp = boost::asio::ip::udp;
    boost::asio::io_service io_service;
    const auto loopback = boost::asio::ip::make_address("127.0.0.1");
    Udp::socket port_reservation(io_service, Udp::endpoint(loopback, 0));
    const auto port = port_reservation.local_endpoint().port();
    port_reservation.close();

    std::vector<Udp::endpoint> senders;
    auto callback = [&](const char*, const size_t&, const Udp::endpoint& sender)
    {
        senders.push_back(sender);
        if (senders.size() == 2)
        {
            io_service.stop();
        }
    };
    std::unique_ptr<UdpBidirectionalSocket> socket;
    if (GetParam())
    {
        socket = std::make_unique<UdpBidirectionalSocket>(io_service, port, callback);
    }
    else
    {
        socket = std::make_unique<UdpBidirectionalSocket>(io_service, "127.0.0.1", port,
                                                          "", false, callback);
    }

    Udp::socket first_client(io_service, Udp::endpoint(loopback, 0));
    Udp::socket second_client(io_service, Udp::endpoint(loopback, 0));
    const std::array<Udp::socket*, 2> clients = {&first_client, &second_client};
    for (auto* client : clients)
    {
        client->send_to(boost::asio::buffer("request", 7), Udp::endpoint(loopback, port));
        client->non_blocking(true);
    }
    io_service.run_for(std::chrono::seconds(2));
    ASSERT_EQ(senders.size(), clients.size());

    const std::array<std::string, 2> replies = {std::string("a\0b", 3), ""};
    socket->sendTo(replies[1], senders[1]);
    socket->sendTo(replies[0], senders[0]);

    // sends are async, so this flushes them before we check the results
    io_service.restart();
    io_service.poll();

    for (size_t i = 0; i < clients.size(); ++i)
    {
        std::array<char, 16> buffer;
        Udp::endpoint source;
        boost::system::error_code error;
        const auto size =
            clients[i]->receive_from(boost::asio::buffer(buffer), source, 0, error);
        ASSERT_FALSE(error) << error.message();
        EXPECT_EQ(std::string(buffer.data(), size), replies[i]);
        EXPECT_EQ(source, Udp::endpoint(loopback, port));
    }
}

INSTANTIATE_TEST_SUITE_P(SocketConstructors, UdpBidirectionalSocketTest, testing::Bool());
