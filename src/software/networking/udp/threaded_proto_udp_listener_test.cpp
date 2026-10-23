#include "software/networking/udp/threaded_proto_udp_listener.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <boost/asio.hpp>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>

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

TEST(ThreadedProtoUdpListenerTest, idle_destruction_keeps_shared_context_alive)
{
    auto io_context       = std::make_shared<ThreadedIoContext>();
    const auto first_port = availablePort();

    {
        ThreadedProtoUdpListener<google::protobuf::Empty> listener(
            io_context, first_port, [](const google::protobuf::Empty&) {});
    }

    const auto second_port = availablePort();
    std::promise<void> received;
    auto received_future = received.get_future();
    ThreadedProtoUdpListener<google::protobuf::Empty> second_listener(
        io_context, second_port,
        [&received](const google::protobuf::Empty&) { received.set_value(); });

    sendPacket(second_port);
    ASSERT_EQ(received_future.wait_for(TEST_TIMEOUT), std::future_status::ready);

    second_listener.close();
}

TEST(ThreadedProtoUdpListenerTest, close_stops_further_callbacks)
{
    auto io_context = std::make_shared<ThreadedIoContext>();
    const auto port = availablePort();
    std::atomic<int> callback_count{0};
    std::promise<void> first_received;
    auto first_received_future = first_received.get_future();

    auto listener = std::make_unique<ThreadedProtoUdpListener<google::protobuf::Empty>>(
        io_context, port,
        [&callback_count, &first_received](const google::protobuf::Empty&)
        {
            if (callback_count.fetch_add(1) == 0)
            {
                first_received.set_value();
            }
        });

    sendPacket(port);
    ASSERT_EQ(first_received_future.wait_for(TEST_TIMEOUT), std::future_status::ready);

    listener->close();
    const int count_at_close = callback_count.load();

    sendPacket(port);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    EXPECT_EQ(callback_count.load(), count_at_close);
}

TEST(ThreadedProtoUdpListenerTest, concurrent_close_is_safe)
{
    auto io_context = std::make_shared<ThreadedIoContext>();
    const auto port = availablePort();

    auto listener = std::make_unique<ThreadedProtoUdpListener<google::protobuf::Empty>>(
        io_context, port, [](const google::protobuf::Empty&) {});

    auto first_close = std::async(std::launch::async, [&listener] { listener->close(); });
    auto second_close =
        std::async(std::launch::async, [&listener] { listener->close(); });

    ASSERT_EQ(first_close.wait_for(TEST_TIMEOUT), std::future_status::ready);
    ASSERT_EQ(second_close.wait_for(TEST_TIMEOUT), std::future_status::ready);
}

TEST(ThreadedProtoUdpListenerTest, close_releases_port)
{
    auto io_context = std::make_shared<ThreadedIoContext>();
    const auto port = availablePort();

    auto listener = std::make_unique<ThreadedProtoUdpListener<google::protobuf::Empty>>(
        io_context, port, [](const google::protobuf::Empty&) {});

    listener->close();

    boost::asio::io_context rebind_context;
    boost::asio::ip::udp::socket rebind_socket(rebind_context);
    rebind_socket.open(boost::asio::ip::udp::v6());
    rebind_socket.set_option(boost::asio::ip::v6_only(false));
    boost::system::error_code error;
    rebind_socket.bind({boost::asio::ip::address_v6::any(), port}, error);
    EXPECT_FALSE(error);
    rebind_socket.close();
}
