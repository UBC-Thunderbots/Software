#include "software/ai/navigator/navigator.h"

#include <google/protobuf/util/message_differencer.h>
#include <gtest/gtest.h>

#include "software/ai/intent/all_intents.h"
#include "software/ai/navigator/path_manager/velocity_obstacle_path_manager.h"
#include "software/ai/navigator/path_planner/no_path_test_path_planner.h"
#include "software/ai/navigator/path_planner/one_point_path_test_path_planner.h"
#include "software/ai/navigator/path_planner/theta_star_path_planner.h"
#include "software/primitive/all_primitives.h"
#include "software/proto/message_translation/proto_creator_primitive_visitor.h"
#include "software/test_util/test_util.h"

class NoPathNavigatorTest : public testing::Test
{
   public:
    NoPathNavigatorTest()
        : robot_navigation_obstacle_factory(RobotNavigationObstacleFactory(
              DynamicParameters->getAIConfig()
                  ->getRobotNavigationObstacleFactoryConfig())),
          navigator(std::make_unique<VelocityObstaclePathManager>(
                        std::make_unique<NoPathTestPathPlanner>(),
                        robot_navigation_obstacle_factory),
                    robot_navigation_obstacle_factory,
                    DynamicParameters->getAIConfig()->getNavigatorConfig()),
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

class ThetaStarNavigatorTest : public testing::Test
{
   public:
    ThetaStarNavigatorTest()
        : robot_navigation_obstacle_factory(RobotNavigationObstacleFactory(
              DynamicParameters->getAIConfig()
                  ->getRobotNavigationObstacleFactoryConfig())),
          navigator(std::make_unique<VelocityObstaclePathManager>(
                        std::make_unique<ThetaStarPathPlanner>(),
                        robot_navigation_obstacle_factory),
                    robot_navigation_obstacle_factory,
                    DynamicParameters->getAIConfig()->getNavigatorConfig())
    {
    }

    RobotNavigationObstacleFactory robot_navigation_obstacle_factory;

    Navigator navigator;
};

TEST_F(ThetaStarNavigatorTest, convert_chip_intent_to_chip_primitive)
{
    World world = ::TestUtil::createBlankTestingWorld();

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(
        std::make_unique<ChipIntent>(0, Point(), Angle::quarter(), 0, 1));

    auto primitive_set_msg = navigator.getAssignedPrimitiveSetMsg(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_set_msg->robot_primitives().size(), 1);

    auto expected_primitive = ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(
        ChipPrimitive(0, Point(), Angle::quarter(), 0));
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_primitive, primitive_set_msg->robot_primitives().at(0)));
}

TEST_F(ThetaStarNavigatorTest, convert_kick_intent_to_kick_primitive)
{
    World world = ::TestUtil::createBlankTestingWorld();

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(
        std::make_unique<KickIntent>(0, Point(), Angle::quarter(), 0, 1));

    auto primitive_set_msg = navigator.getAssignedPrimitiveSetMsg(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_set_msg->robot_primitives().size(), 1);

    auto expected_primitive = ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(
        KickPrimitive(0, Point(), Angle::quarter(), 0));
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_primitive, primitive_set_msg->robot_primitives().at(0)));
}

TEST_F(ThetaStarNavigatorTest, convert_spinning_move_intent_to_spinning_move_primitive)
{
    World world = ::TestUtil::createBlankTestingWorld();

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(
        std::make_unique<SpinningMoveIntent>(0, Point(), AngularVelocity::full(), 1, 0));

    auto primitive_set_msg = navigator.getAssignedPrimitiveSetMsg(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_set_msg->robot_primitives().size(), 1);

    auto expected_primitive = ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(
        SpinningMovePrimitive(0, Point(), AngularVelocity::full(), 1));
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_primitive, primitive_set_msg->robot_primitives().at(0)));
}

TEST_F(ThetaStarNavigatorTest, convert_stop_intent_to_stop_primitive)
{
    World world = ::TestUtil::createBlankTestingWorld();

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(std::make_unique<StopIntent>(0, false, 1));

    auto primitive_set_msg = navigator.getAssignedPrimitiveSetMsg(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_set_msg->robot_primitives().size(), 1);

    auto expected_primitive =
        ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(StopPrimitive(0, false));
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_primitive, primitive_set_msg->robot_primitives().at(0)));
}

TEST_F(ThetaStarNavigatorTest, convert_multiple_intents_to_primitives)
{
    World world = ::TestUtil::createBlankTestingWorld();

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(std::make_unique<StopIntent>(0, false, 1));
    intents.emplace_back(std::make_unique<StopIntent>(1, false, 1));

    auto primitive_set_msg = navigator.getAssignedPrimitiveSetMsg(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_set_msg->robot_primitives().size(), 2);

    auto expected_stop_primitive_1 =
        ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(StopPrimitive(0, false));
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_stop_primitive_1, primitive_set_msg->robot_primitives().at(0)));

    auto expected_stop_primitive_2 =
        ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(StopPrimitive(1, false));
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_stop_primitive_2, primitive_set_msg->robot_primitives().at(1)));
}

TEST(NavigatorTest, move_intent_with_one_point_path_test_path_planner)
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

    Navigator navigator(
        std::make_unique<VelocityObstaclePathManager>(
            std::make_unique<OnePointPathTestPathPlanner>(),
            RobotNavigationObstacleFactory(
                DynamicParameters->getAIConfig()
                    ->getRobotNavigationObstacleFactoryConfig())),
        RobotNavigationObstacleFactory(
            DynamicParameters->getAIConfig()->getRobotNavigationObstacleFactoryConfig()),
        std::make_shared<NavigatorConfig>());

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(std::make_unique<MoveIntent>(
        0, poi, Angle::zero(), 0, 0, DribblerEnable::OFF, MoveType::NORMAL,
        AutochickType::NONE, BallCollisionType::AVOID));

    auto primitive_set_msg = navigator.getAssignedPrimitiveSetMsg(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_set_msg->robot_primitives().size(), 1);

    auto expected_primitive = ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(
        MovePrimitive(0, poi, Angle::zero(), 0, DribblerEnable::OFF, MoveType::NORMAL,
                      AutochickType::NONE));
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
        0, Point(), Angle::zero(), 0, 0, DribblerEnable::OFF, MoveType::NORMAL,
        AutochickType::NONE, BallCollisionType::AVOID));

    auto primitive_set_msg = navigator.getAssignedPrimitiveSetMsg(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_set_msg->robot_primitives().size(), 1);

    auto expected_primitive =
        ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(StopPrimitive(0, false));
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_primitive, primitive_set_msg->robot_primitives().at(0)));
}

TEST_F(NoPathNavigatorTest,
       calculateTransitionSpeedBetweenSegments_tests_parallel_segments)
{
    Point testp1, testp2, testp3;
    double final_speed;
    // case 1
    testp1      = Point(1, 0);
    testp2      = Point(2, 0);
    testp3      = Point(3, 0);
    final_speed = 2.2;
    EXPECT_DOUBLE_EQ(final_speed, navigator.calculateTransitionSpeedBetweenSegments(
                                      testp1, testp2, testp3, final_speed));

    // case 2
    testp1      = Point(1, 1);
    testp2      = Point(1, 2);
    testp3      = Point(1, 3);
    final_speed = -2.2;
    EXPECT_DOUBLE_EQ(final_speed, navigator.calculateTransitionSpeedBetweenSegments(
                                      testp1, testp2, testp3, final_speed));
}

TEST_F(NoPathNavigatorTest,
       calculateTransitionSpeedBetweenSegments_tests_opposite_segments)
{
    Point testp1, testp2, testp3;
    double final_speed;
    // unequal segment length
    testp1      = Point(1, 0);
    testp2      = Point(2, 0);
    testp3      = Point(0, 0);
    final_speed = 0;
    EXPECT_DOUBLE_EQ(final_speed, navigator.calculateTransitionSpeedBetweenSegments(
                                      testp1, testp2, testp3, final_speed));

    // equal segment length
    testp1      = Point(1, 1);
    testp2      = Point(1, 2);
    testp3      = Point(1, 1);
    final_speed = 0;
    EXPECT_DOUBLE_EQ(final_speed, navigator.calculateTransitionSpeedBetweenSegments(
                                      testp1, testp2, testp3, final_speed));
}

TEST_F(NoPathNavigatorTest,
       calculateTransitionSpeedBetweenSegments_tests_perpendicular_segments)
{
    Point testp1, testp2, testp3;
    double final_speed;
    // case 1
    testp1      = Point(0, 1);
    testp2      = Point(1, 1);
    testp3      = Point(1, 2);
    final_speed = -2.2;
    EXPECT_DOUBLE_EQ(0, navigator.calculateTransitionSpeedBetweenSegments(
                            testp1, testp2, testp3, final_speed));

    // case 2
    testp1      = Point(1, 0);
    testp2      = Point(2, 0);
    testp3      = Point(2, 1);
    final_speed = 2.2;
    EXPECT_DOUBLE_EQ(0, navigator.calculateTransitionSpeedBetweenSegments(
                            testp1, testp2, testp3, final_speed));
}


TEST_F(NoPathNavigatorTest,
       calculateTransitionSpeedBetweenSegments_tests_nan_corner_cases)
{
    Point testp1, testp2, testp3;
    double final_speed;
    // case 1
    testp1      = Point(0, 1);
    testp2      = Point(0, 1);
    testp3      = Point(1, 2);
    final_speed = -2.2;
    EXPECT_FALSE(isnormal(navigator.calculateTransitionSpeedBetweenSegments(
        testp1, testp2, testp3, final_speed)));

    // case 2
    testp1      = Point(1, 0);
    testp2      = Point(2, 0);
    testp3      = Point(2, 0);
    final_speed = 2.2;
    EXPECT_FALSE(isnormal(navigator.calculateTransitionSpeedBetweenSegments(
        testp1, testp2, testp3, final_speed)));
}
