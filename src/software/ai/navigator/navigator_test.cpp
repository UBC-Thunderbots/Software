#include "software/ai/navigator/navigator.h"

#include <google/protobuf/util/message_differencer.h>
#include <gtest/gtest.h>

#include "software/ai/intent/all_intents.h"
#include "software/ai/navigator/path_manager/velocity_obstacle_path_manager.h"
#include "software/ai/navigator/path_planner/no_path_test_path_planner.h"
#include "software/ai/navigator/path_planner/one_point_path_test_path_planner.h"
#include "software/ai/navigator/path_planner/theta_star_path_planner.h"
#include "software/test_util/test_util.h"

class NavigatorTest : public testing::Test
{
   protected:
    RobotConstants_t robot_constants = create2015RobotConstants();
};

class NoPathNavigatorTest : public NavigatorTest
{
   public:
    NoPathNavigatorTest()
        : robot_navigation_obstacle_factory(RobotNavigationObstacleFactory()),
          navigator(std::make_unique<VelocityObstaclePathManager>(
                        std::make_unique<NoPathTestPathPlanner>(),
                        robot_navigation_obstacle_factory),
                    robot_navigation_obstacle_factory),
          current_time(Timestamp::fromSeconds(123)),
          field(Field::createSSLDivisionBField()),
          ball(Ball(Point(1, 2), Vector(-0.3, 0), current_time)),
          friendly_team(Team(Duration::fromMilliseconds(1000))),
          enemy_team(Team(Duration::fromMilliseconds(1000)))
    {
    }

    RobotNavigationObstacleFactory robot_navigation_obstacle_factory;

    // The navigator under test
    Navigator navigator;

    Timestamp current_time;
    Field field;
    Ball ball;
    Team friendly_team;
    Team enemy_team;
};

class ThetaStarNavigatorTest : public NavigatorTest
{
   public:
    ThetaStarNavigatorTest()
        : robot_navigation_obstacle_factory(RobotNavigationObstacleFactory()),
          navigator(std::make_unique<VelocityObstaclePathManager>(
                        std::make_unique<NoPathTestPathPlanner>(),
                        robot_navigation_obstacle_factory),
                    robot_navigation_obstacle_factory)
    {
    }

    RobotNavigationObstacleFactory robot_navigation_obstacle_factory;

    Navigator navigator;
};

TEST_F(ThetaStarNavigatorTest, convert_chip_intent_to_move_with_autochip_primitive)
{
    World world = ::TestUtil::createBlankTestingWorld();

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(
        std::make_unique<ChipIntent>(0, Point(), Angle::quarter(), 0, robot_constants));

    auto primitive_set_msg = navigator.getAssignedPrimitives(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_set_msg->robot_primitives().size(), 1);

    auto expected_primitive = *createMovePrimitive(
        Point(), 0, Angle::quarter(), TbotsProto::DribblerMode::OFF,
        AutoChipOrKick{
            AutoChipOrKickMode::AUTOCHIP,
            0,
        },
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0, robot_constants);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_primitive, primitive_set_msg->robot_primitives().at(0)));
}

TEST_F(ThetaStarNavigatorTest, convert_kick_intent_to_move_with_autokick_primitive)
{
    World world = ::TestUtil::createBlankTestingWorld();

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(
        std::make_unique<KickIntent>(0, Point(), Angle::quarter(), 0, robot_constants));

    auto primitive_set_msg = navigator.getAssignedPrimitives(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_set_msg->robot_primitives().size(), 1);

    auto expected_primitive = *createMovePrimitive(
        Point(), 0, Angle::quarter(), TbotsProto::DribblerMode::OFF,
        AutoChipOrKick{
            AutoChipOrKickMode::AUTOKICK,
            0,
        },
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0, robot_constants);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_primitive, primitive_set_msg->robot_primitives().at(0)));
}

TEST_F(ThetaStarNavigatorTest, convert_stop_intent_to_stop_primitive)
{
    World world = ::TestUtil::createBlankTestingWorld();

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(std::make_unique<StopIntent>(0, false));

    auto primitive_set_msg = navigator.getAssignedPrimitives(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_set_msg->robot_primitives().size(), 1);

    auto expected_primitive = *createStopPrimitive(false);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_primitive, primitive_set_msg->robot_primitives().at(0)));
}

TEST_F(ThetaStarNavigatorTest, convert_multiple_intents_to_primitives)
{
    World world = ::TestUtil::createBlankTestingWorld();

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(std::make_unique<StopIntent>(0, false));
    intents.emplace_back(std::make_unique<StopIntent>(1, false));

    auto primitive_set_msg = navigator.getAssignedPrimitives(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_set_msg->robot_primitives().size(), 2);

    auto expected_stop_primitive_1 = *createStopPrimitive(false);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_stop_primitive_1, primitive_set_msg->robot_primitives().at(0)));

    auto expected_stop_primitive_2 = *createStopPrimitive(false);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_stop_primitive_2, primitive_set_msg->robot_primitives().at(1)));
}

TEST_F(NavigatorTest, move_intent_with_one_point_path_test_path_planner)
{
    Point poi = Point(2, -3);
    Timestamp current_time(Timestamp::fromSeconds(123));
    Ball ball(Point(1, 2), Vector(-0.3, 0), current_time);
    Team friendly_team(Duration::fromMilliseconds(1000));
    Team enemy_team(Duration::fromMilliseconds(1000));

    // An arbitrary fixed point in time
    // We use this fixed point in time to make the tests deterministic.
    Field field = Field::createSSLDivisionBField();

    Robot friendly_robot_0 = Robot(0, poi, Vector(-1, -2), Angle::half(),
                                   AngularVelocity::threeQuarter(), current_time);

    Robot friendly_robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                                   AngularVelocity::zero(), current_time);

    friendly_team.updateRobots({friendly_robot_0, friendly_robot_1});
    friendly_team.assignGoalie(1);

    Robot enemy_robot_0 = Robot(0, Point(0.5, -2.5), Vector(), Angle::fromRadians(1),
                                AngularVelocity::fromRadians(2), current_time);

    Robot enemy_robot_1 = Robot(1, Point(), Vector(-0.5, 4), Angle::quarter(),
                                AngularVelocity::half(), current_time);

    enemy_team.updateRobots({enemy_robot_0, enemy_robot_1});
    enemy_team.assignGoalie(0);

    // Construct the world with arguments
    World world = World(field, ball, friendly_team, enemy_team);

    Navigator navigator(std::make_unique<VelocityObstaclePathManager>(
                            std::make_unique<OnePointPathTestPathPlanner>(),
                            RobotNavigationObstacleFactory()),
                        RobotNavigationObstacleFactory());

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(std::make_unique<MoveIntent>(
        0, poi, Angle::zero(), 0, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::AVOID, AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0, robot_constants));

    auto primitive_set_msg = navigator.getAssignedPrimitives(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_set_msg->robot_primitives().size(), 1);

    auto expected_primitive = *createMovePrimitive(
        poi, 0, Angle::zero(), TbotsProto::DribblerMode::OFF,
        AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0, robot_constants);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_primitive, primitive_set_msg->robot_primitives().at(0)));
}

TEST_F(NoPathNavigatorTest, move_intent_with_no_path_test_path_planner)
{
    Robot friendly_robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                                   AngularVelocity::threeQuarter(), current_time);

    Robot friendly_robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                                   AngularVelocity::zero(), current_time);

    friendly_team.updateRobots({friendly_robot_0, friendly_robot_1});
    friendly_team.assignGoalie(1);

    Robot enemy_robot_0 = Robot(0, Point(0.5, -2.5), Vector(), Angle::fromRadians(1),
                                AngularVelocity::fromRadians(2), current_time);

    Robot enemy_robot_1 = Robot(1, Point(), Vector(-0.5, 4), Angle::quarter(),
                                AngularVelocity::half(), current_time);

    enemy_team.updateRobots({enemy_robot_0, enemy_robot_1});
    enemy_team.assignGoalie(0);

    // Construct the world with arguments
    World world = World(field, ball, friendly_team, enemy_team);

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(std::make_unique<MoveIntent>(
        0, Point(), Angle::zero(), 0, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::AVOID, AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0, robot_constants));

    auto primitive_set_msg = navigator.getAssignedPrimitives(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_set_msg->robot_primitives().size(), 1);

    auto expected_primitive = *createStopPrimitive(false);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_primitive, primitive_set_msg->robot_primitives().at(0)));
}
