#include "software/networking/udp/threaded_proto_udp_listener.hpp"

#include <gtest/gtest.h>

#include <boost/asio.hpp>
#include <chrono>
#include <future>
#include <memory>
#include <string>

#include "google/protobuf/empty.pb.h"
#include "shared/constants.h"
#include "software/networking/tbots_network_exception.h"
#include "software/networking/udp/threaded_io_context.h"

namespace
{
constexpr auto TEST_TIMEOUT = std::chrono::seconds(1);

unsigned short availablePort()
{
    boost::asio::io_context io_context;
    boost::asio::ip::udp::socket socket(io_context);
    socket.open(boost::asio::ip::udp::v6());
    socket.bind({boost::asio::ip::address_v6::loopback(), 0});

    const auto port = socket.local_endpoint().port();
    socket.close();
    return port;
}

void sendPacket(unsigned short port)
{
    boost::asio::io_context io_context;
    boost::asio::ip::udp::socket socket(io_context);
    socket.open(boost::asio::ip::udp::v6());

    const auto endpoint =
        boost::asio::ip::udp::endpoint(boost::asio::ip::address_v6::loopback(), port);
    const std::string packet = "test";
    socket.send_to(boost::asio::buffer(packet), endpoint);
}
}  // namespace

TEST(ThreadedProtoUdpListenerTest, error_finding_local_ip_address)
{
    auto io_context = std::make_shared<ThreadedIoContext>();
    EXPECT_THROW(ThreadedProtoUdpListener<google::protobuf::Empty>(
                     io_context, "224.5.23.1", 40000, "interfacemcinterfaceface",
                     [](const auto&) {}, true),
                 TbotsNetworkException);
}

TEST(ThreadedProtoUdpListenerTest, error_creating_socket)
{
    auto io_context = std::make_shared<ThreadedIoContext>();
    // This will always fail because it requires root privileges to open this port
    EXPECT_THROW(
        ThreadedProtoUdpListener<google::protobuf::Empty>(
            io_context, "224.5.23.1", 1023, LOOPBACK_INTERFACE, [](const auto&) {}, true),
        TbotsNetworkException);
}

TEST(ThreadedProtoUdpListenerTest, no_error_creating_socket)
{
    auto io_context = std::make_shared<ThreadedIoContext>();
    ThreadedProtoUdpListener<google::protobuf::Empty>(
        io_context, "224.5.23.0", 40000, LOOPBACK_INTERFACE, [](const auto&) {}, true);
}

TEST(ThreadedProtoUdpListenerTest, multiple_listeners_share_io_context)
{
    auto io_context = std::make_shared<ThreadedIoContext>();
    ThreadedProtoUdpListener<google::protobuf::Empty> first_listener(io_context, 40001,
                                                                     [](const auto&) {});
    ThreadedProtoUdpListener<google::protobuf::Empty> second_listener(io_context, 40002,
                                                                      [](const auto&) {});

    first_listener.close();
    second_listener.close();
}

TEST(ThreadedProtoUdpListenerTest, destruction_waits_for_active_callback)
{
    auto io_context = std::make_shared<ThreadedIoContext>();
    std::promise<void> callback_started;
    auto callback_started_future = callback_started.get_future();
    std::promise<void> release_callback;
    auto release_callback_future = release_callback.get_future().share();
    const auto port              = availablePort();

    auto listener = std::make_unique<ThreadedProtoUdpListener<google::protobuf::Empty>>(
        io_context, port,
        [&callback_started, release_callback_future](const google::protobuf::Empty&)
        {
            callback_started.set_value();
            release_callback_future.wait();
        });

    sendPacket(port);
    ASSERT_EQ(callback_started_future.wait_for(TEST_TIMEOUT), std::future_status::ready);

    auto destruction = std::async(std::launch::async, [&listener] { listener.reset(); });
    const bool destroyed_while_callback_was_active =
        destruction.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready;

    release_callback.set_value();

    ASSERT_EQ(destruction.wait_for(TEST_TIMEOUT), std::future_status::ready);
    EXPECT_FALSE(destroyed_while_callback_was_active);
}
