#include "software/networking/udp/threaded_proto_udp_sender.hpp"

#include <gtest/gtest.h>

#include "google/protobuf/empty.pb.h"

TEST(ThreadedProtoUdpSenderTest, error_finding_local_ip_address)
{
    std::optional<std::string> error;
    ThreadedProtoUdpSender<google::protobuf::Empty>(
        "224.5.23.1", 40000, "interfacemcinterfaceface", true, error);
    EXPECT_TRUE(error.has_value());
}

TEST(ThreadedProtoUdpSenderTest, no_error_creating_socket)
{
    std::optional<std::string> error;
    ThreadedProtoUdpSender<google::protobuf::Empty>("224.5.23.1", 40000, "lo", true,
                                                    error);
    EXPECT_FALSE(error.has_value());
}
