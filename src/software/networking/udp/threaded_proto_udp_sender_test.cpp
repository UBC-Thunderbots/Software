#include "software/networking/udp/threaded_proto_udp_sender.hpp"

#include <gtest/gtest.h>

#include "google/protobuf/empty.pb.h"
#include "shared/constants.h"
#include "software/networking/tbots_network_exception.h"
#include "software/networking/udp/threaded_io_context.h"

TEST(ThreadedProtoUdpSenderTest, error_finding_local_ip_address)
{
    auto io_context = std::make_shared<ThreadedIoContext>();
    EXPECT_THROW(ThreadedProtoUdpSender<google::protobuf::Empty>(
                     io_context, "224.5.23.1", 40000, "interfacemcinterfaceface", true),
                 TbotsNetworkException);
}

TEST(ThreadedProtoUdpSenderTest, no_error_creating_socket)
{
    auto io_context = std::make_shared<ThreadedIoContext>();
    ThreadedProtoUdpSender<google::protobuf::Empty>(io_context, "224.5.23.1", 40000,
                                                    LOOPBACK_INTERFACE, true);
}
