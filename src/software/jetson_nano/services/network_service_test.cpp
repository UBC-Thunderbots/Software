#include "software/jetson_nano/services/network.h"
#include "software/backend/simulator_backend.h"

#include <gtest/gtest.h>

TEST(NetworkTest, test_receive)
{
    
    std::string YOUR_INTERFACE = "enp0s5";
    auto network_service = NetworkService("ff02::c3d0:42d2:bb01%" + YOUR_INTERFACE, VISION_PORT,PRIMITIVE_PORT, ROBOT_STATUS_PORT, true);
    auto sender = std::make_unique<ThreadedProtoUdpSender<TbotsProto::PrimitiveSet>>("ff02::c3d0:42d2:bb01%" + YOUR_INTERFACE, PRIMITIVE_PORT, true);
    TbotsProto::PrimitiveSet message;

    sender->sendProto(message);
    sender->sendProto(message);
    sender->sendProto(message);
    sender->sendProto(message);
    TbotsProto::RobotStatus status;
    network_service.poll(status);


    EXPECT_EQ(180, 180);
}