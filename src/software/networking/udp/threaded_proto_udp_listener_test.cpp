#include "google/protobuf/empty.pb.h"
#include "software/networking/udp/threaded_proto_udp_listener.hpp"

#include <gtest/gtest.h>

TEST(ThreadedProtoUdpListenerTest, error_finding_ip_address)
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
