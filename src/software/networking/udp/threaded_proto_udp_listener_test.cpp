#include "software/networking/udp/threaded_proto_udp_listener.hpp"

#include <gtest/gtest.h>

#include "google/protobuf/empty.pb.h"
#include "software/networking/tbots_network_exception.h"

TEST(ThreadedProtoUdpListenerTest, error_finding_local_ip_address)
{
    EXPECT_THROW(
        ThreadedProtoUdpListener<google::protobuf::Empty>(
            "224.5.23.1", 40000, "interfacemcinterfaceface", [](const auto&) {}, true),
        TbotsNetworkException);
}

TEST(ThreadedProtoUdpListenerTest, error_creating_socket)
{
    // This will always fail because it requires root privileges to open this port
    EXPECT_THROW(ThreadedProtoUdpListener<google::protobuf::Empty>(
                     "224.5.23.1", 1023, "lo", [](const auto&) {}, true),
                 TbotsNetworkException);
}

TEST(ThreadedProtoUdpListenerTest, no_error_creating_socket)
{
    EXPECT_THROW(ThreadedProtoUdpListener<google::protobuf::Empty>(
                     "224.5.23.0", 40000, "lo", [](const auto&) {}, true),
                 TbotsNetworkException);
}
