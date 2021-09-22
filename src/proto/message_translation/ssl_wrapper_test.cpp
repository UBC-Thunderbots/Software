#include "proto/message_translation/ssl_wrapper.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"
#include "software/world/world.h"

TEST(SSLWrapperTest, test_create_empty_message)
{
    auto wrapper_packet = createSSLWrapperPacket(nullptr, nullptr);
    EXPECT_FALSE(wrapper_packet->has_geometry());
    EXPECT_FALSE(wrapper_packet->has_detection());
}

TEST(SSLWrapperTest, test_create_wrapper_only_with_geometry)
{
    auto geometry_data = std::make_unique<SSLProto::SSL_GeometryData>();

    auto wrapper_packet = createSSLWrapperPacket(std::move(geometry_data), nullptr);
    EXPECT_TRUE(wrapper_packet->has_geometry());
    EXPECT_FALSE(wrapper_packet->has_detection());
}

TEST(SSLWrapperTest, test_create_wrapper_only_with_detection_frame)
{
    auto detection_frame = std::make_unique<SSLProto::SSL_DetectionFrame>();

    auto wrapper_packet = createSSLWrapperPacket(nullptr, std::move(detection_frame));
    EXPECT_FALSE(wrapper_packet->has_geometry());
    EXPECT_TRUE(wrapper_packet->has_detection());
}

TEST(SSLWrapperTest, test_create_wrapper_with_all_data)
{
    auto geometry_data   = std::make_unique<SSLProto::SSL_GeometryData>();
    auto detection_frame = std::make_unique<SSLProto::SSL_DetectionFrame>();

    auto wrapper_packet =
        createSSLWrapperPacket(std::move(geometry_data), std::move(detection_frame));
    EXPECT_TRUE(wrapper_packet->has_geometry());
    EXPECT_TRUE(wrapper_packet->has_detection());
}

TEST(SSLWrapperTest, test_create_wrapper_with_world_friendly_yellow)
{
    Robot yellow_robot1(1, Point(1, 0), Vector(0, 0), Angle::quarter(),
                        AngularVelocity::half(), Timestamp());
    Robot yellow_robot2(2, Point(0, 0), Vector(3, 0), Angle::half(),
                        AngularVelocity::quarter(), Timestamp());
    std::vector<Robot> yellow_robots = {yellow_robot1, yellow_robot2};

    Robot blue_robot1(3, Point(1, 0), Vector(0, 0), Angle::quarter(),
                      AngularVelocity::half(), Timestamp());
    Robot blue_robot2(4, Point(0, 0), Vector(3, 0), Angle::half(),
                      AngularVelocity::quarter(), Timestamp());
    Robot blue_robot3(5, Point(-1, -1), Vector(0, 2), Angle::half(),
                      AngularVelocity::quarter(), Timestamp());
    std::vector<Robot> blue_robots = {blue_robot1, blue_robot2, blue_robot3};

    TeamColour friendly_team_colour = TeamColour::YELLOW;

    World world = TestUtil::createBlankTestingWorld();
    world.updateFriendlyTeamState(Team(yellow_robots));
    world.updateEnemyTeamState(Team(blue_robots));

    auto wrapper_packet_ptr = createSSLWrapperPacket(world, friendly_team_colour);

    EXPECT_EQ(2, wrapper_packet_ptr->detection().robots_yellow_size());
    EXPECT_EQ(3, wrapper_packet_ptr->detection().robots_blue_size());
}

TEST(SSLWrapperTest, test_create_wrapper_with_world_friendly_not_yellow)
{
    Robot yellow_robot1(1, Point(1, 0), Vector(0, 0), Angle::quarter(),
                        AngularVelocity::half(), Timestamp());
    Robot yellow_robot2(2, Point(0, 0), Vector(3, 0), Angle::half(),
                        AngularVelocity::quarter(), Timestamp());
    std::vector<Robot> yellow_robots = {yellow_robot1, yellow_robot2};

    Robot blue_robot1(3, Point(1, 0), Vector(0, 0), Angle::quarter(),
                      AngularVelocity::half(), Timestamp());
    Robot blue_robot2(4, Point(0, 0), Vector(3, 0), Angle::half(),
                      AngularVelocity::quarter(), Timestamp());
    Robot blue_robot3(5, Point(-1, -1), Vector(0, 2), Angle::half(),
                      AngularVelocity::quarter(), Timestamp());
    std::vector<Robot> blue_robots = {blue_robot1, blue_robot2, blue_robot3};

    TeamColour friendly_team_colour = TeamColour::BLUE;

    World world = TestUtil::createBlankTestingWorld();
    world.updateFriendlyTeamState(Team(blue_robots));
    world.updateEnemyTeamState(Team(yellow_robots));

    auto wrapper_packet_ptr = createSSLWrapperPacket(world, friendly_team_colour);

    EXPECT_EQ(2, wrapper_packet_ptr->detection().robots_yellow_size());
    EXPECT_EQ(3, wrapper_packet_ptr->detection().robots_blue_size());
}
