#include "software/networking/udp/threaded_proto_udp_listener.hpp"

#include <gtest/gtest.h>

TEST(ThreadedProtoUdpListener, error_finding_ip_address)
{
    std::optional<std::string> error;
    ThreadedProtoUdpListener("192.168.0.2", 40000, "interfacemcinterfaceface", [](const auto&){}, true, error);
    EXPECT_TRUE(error.has_value());
}
