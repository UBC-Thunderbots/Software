#include "software/networking/udp/threaded_proto_udp_sender.hpp"

#include <gtest/gtest.h>

#include "google/protobuf/empty.pb.h"
#include "software/networking/tbots_network_exception.h"

TEST(ThreadedProtoUdpSenderTest, error_finding_local_ip_address)
{
    EXPECT_THROW(ThreadedProtoUdpSender<google::protobuf::Empty>(
                     "224.5.23.1", 40000, "interfacemcinterfaceface", true),
                 TbotsNetworkException);
}

TEST(ThreadedProtoUdpSenderTest, no_error_creating_socket)
{
    ThreadedProtoUdpSender<google::protobuf::Empty>("224.5.23.1", 40000, "lo", true);
}
