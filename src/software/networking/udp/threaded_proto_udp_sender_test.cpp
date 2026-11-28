#include "software/networking/udp/threaded_proto_udp_sender.hpp"

#include <gtest/gtest.h>

#include <chrono>

#include "google/protobuf/empty.pb.h"
#include "shared/constants.h"
#include "software/networking/tbots_network_exception.h"

TEST(ThreadedProtoUdpSenderTest, error_finding_local_ip_address)
{
    EXPECT_THROW(ThreadedProtoUdpSender<google::protobuf::Empty>(
                     "224.5.23.1", 40000, "interfacemcinterfaceface", true),
                 TbotsNetworkException);
}

TEST(ThreadedProtoUdpSenderTest, no_error_creating_socket)
{
    ThreadedProtoUdpSender<google::protobuf::Empty>("224.5.23.1", 40000,
                                                    LOOPBACK_INTERFACE, true);
}

TEST(ThreadedProtoUdpSenderTest, sends_multicast_on_the_requested_interface)
{
    using namespace boost::asio;
    io_service io;
    ip::udp::socket receiver(io, ip::udp::endpoint(ip::udp::v4(), 0));
    const auto group = ip::make_address_v4("224.5.23.2");
    receiver.set_option(
        ip::multicast::join_group(group, ip::make_address_v4("127.0.0.1")));
    ThreadedProtoUdpSender<google::protobuf::Empty> sender(
        group.to_string(), receiver.local_endpoint().port(), LOOPBACK_INTERFACE, true);

    std::array<char, 32> bytes;
    ip::udp::endpoint source;
    bool received = false;
    receiver.async_receive_from(buffer(bytes), source,
                                [&](const boost::system::error_code& error, size_t size)
                                {
                                    EXPECT_FALSE(error) << error.message();
                                    EXPECT_EQ(size, 0);
                                    EXPECT_TRUE(source.address().is_loopback());
                                    received = true;
                                    io.stop();
                                });
    sender.sendProto(google::protobuf::Empty());
    io.run_for(std::chrono::seconds(1));
    EXPECT_TRUE(received);
}
