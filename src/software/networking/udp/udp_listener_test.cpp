#include "software/networking/udp/udp_listener.h"

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

class UdpListenerTest : public testing::TestWithParam<bool>
{
};

TEST_P(UdpListenerTest, preserves_each_packets_sender_and_payload)
{
    using Udp = boost::asio::ip::udp;
    boost::asio::io_service io_service;
    const auto loopback = boost::asio::ip::make_address("127.0.0.1");
    Udp::socket port_reservation(io_service, Udp::endpoint(loopback, 0));
    const auto port = port_reservation.local_endpoint().port();
    port_reservation.close();

    std::vector<std::pair<std::string, Udp::endpoint>> packets;
    auto callback =
        [&](const char* data, const size_t& size, const Udp::endpoint& sender_endpoint)
    {
        packets.emplace_back(std::string(data, size), sender_endpoint);
        if (packets.size() == 3)
        {
            io_service.stop();
        }
    };
    std::unique_ptr<UdpListener> listener;
    if (GetParam())
    {
        listener = std::make_unique<UdpListener>(io_service, port, callback);
    }
    else
    {
        listener = std::make_unique<UdpListener>(io_service, "127.0.0.1", port, "", false,
                                                 callback);
    }

    Udp::socket first_sender(io_service, Udp::endpoint(loopback, 0));
    Udp::socket second_sender(io_service, Udp::endpoint(loopback, 0));
    const std::array<std::string, 3> payloads = {"first", "second", "third"};
    const std::array<Udp::socket*, 3> senders = {&first_sender, &second_sender,
                                                 &first_sender};
    for (size_t i = 0; i < payloads.size(); ++i)
    {
        senders[i]->send_to(boost::asio::buffer(payloads[i]),
                            Udp::endpoint(loopback, port));
    }

    io_service.run_for(std::chrono::seconds(2));
    ASSERT_EQ(packets.size(), payloads.size());
    for (size_t i = 0; i < packets.size(); ++i)
    {
        EXPECT_EQ(packets[i].first, payloads[i]);
        EXPECT_EQ(packets[i].second.port(), senders[i]->local_endpoint().port());
        EXPECT_EQ(packets[i].second.address().to_string(),
                  GetParam() ? "::ffff:127.0.0.1" : "127.0.0.1");
    }
}

INSTANTIATE_TEST_SUITE_P(SocketConstructors, UdpListenerTest, testing::Bool());
