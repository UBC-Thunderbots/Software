#include "software/networking/udp/threaded_proto_udp_listener.hpp"

#include <gtest/gtest.h>

#include "google/protobuf/empty.pb.h"
#include "shared/constants.h"
#include "software/networking/tbots_network_exception.h"
#include "software/networking/udp/threaded_io_context.h"

TEST(ThreadedProtoUdpListenerTest, error_finding_local_ip_address)
{
    auto io_context = std::make_shared<ThreadedIoContext>();
    EXPECT_THROW(
        ThreadedProtoUdpListener<google::protobuf::Empty>(
            io_context, "224.5.23.1", 40000, "interfacemcinterfaceface", [](const auto&) {},
            true),
        TbotsNetworkException);
}

TEST(ThreadedProtoUdpListenerTest, error_creating_socket)
{
    auto io_context = std::make_shared<ThreadedIoContext>();
    // This will always fail because it requires root privileges to open this port
    EXPECT_THROW(ThreadedProtoUdpListener<google::protobuf::Empty>(
                     io_context, "224.5.23.1", 1023, LOOPBACK_INTERFACE, [](const auto&) {},
                     true),
                 TbotsNetworkException);
}

TEST(ThreadedProtoUdpListenerTest, no_error_creating_socket)
{
    auto io_context = std::make_shared<ThreadedIoContext>();
    ThreadedProtoUdpListener<google::protobuf::Empty>(
        io_context, "224.5.23.0", 40000, LOOPBACK_INTERFACE, [](const auto&) {}, true);
}
