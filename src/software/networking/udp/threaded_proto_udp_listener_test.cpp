#include "software/networking/udp/threaded_proto_udp_listener.hpp"

#include "google/protobuf/empty.pb.h"

#include <gtest/gtest.h>

TEST(ThreadedProtoUdpListenerTest, error_finding_local_ip_address)
{
    std::optional<std::string> error;
    ThreadedProtoUdpListener<google::protobuf::Empty>("224.5.23.1", 40000, "interfacemcinterfaceface", [](const auto&){}, true, error);
    EXPECT_TRUE(error.has_value());
}

TEST(ThreadedProtoUdpListenerTest, error_creating_socket)
{
    std::optional<std::string> error;
    ThreadedProtoUdpListener<google::protobuf::Empty>("224.5.23.1", 1023, "lo", [](const auto&){}, true, error);
    EXPECT_TRUE(error.has_value());
}

TEST(ThreadedProtoUdpListenerTest, no_error_creating_socket)
{
    std::optional<std::string> error;
    ThreadedProtoUdpListener<google::protobuf::Empty>("224.5.23.0", 40000, "lo", [](const auto&){}, true, error);
    EXPECT_FALSE(error.has_value());
}
