#include "software/ai/navigator/navigator.h"

#include <gtest/gtest.h>

#include "software/ai/intent/all_intents.h"
#include "software/ai/navigator/path_manager/velocity_obstacle_path_manager.h"
#include "software/ai/navigator/path_planner/no_path_test_path_planner.h"
#include "software/ai/navigator/path_planner/one_point_path_test_path_planner.h"
#include "software/ai/navigator/path_planner/theta_star_path_planner.h"
#include "software/primitive/all_primitives.h"
#include "software/test_util/test_util.h"

class NoPathNavigatorTest : public testing::Test
{
   public:
    NoPathNavigatorTest()
        : robot_navigation_obstacle_factory(RobotNavigationObstacleFactory(
              Util::DynamicParameters->getAIConfig()
                  ->getRobotNavigationObstacleFactoryConfig())),
          navigator(std::make_unique<VelocityObstaclePathManager>(
                        std::make_unique<NoPathTestPathPlanner>(),
                        robot_navigation_obstacle_factory),
                    robot_navigation_obstacle_factory,
                    Util::DynamicParameters->getAIConfig()->getNavigatorConfig()),
          current_time(Timestamp::fromSeconds(123)),
          field(::TestUtil::createSSLDivBField()),
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
              Util::DynamicParameters->getAIConfig()
                  ->getRobotNavigationObstacleFactoryConfig())),
          navigator(std::make_unique<VelocityObstaclePathManager>(
                        std::make_unique<ThetaStarPathPlanner>(),
                        robot_navigation_obstacle_factory),
                    robot_navigation_obstacle_factory,
                    Util::DynamicParameters->getAIConfig()->getNavigatorConfig())
    {
    }

    RobotNavigationObstacleFactory robot_navigation_obstacle_factory;

    Navigator navigator;
};

TEST_F(ThetaStarNavigatorTest, convert_catch_intent_to_catch_primitive)
{
    World world = ::TestUtil::createBlankTestingWorld();

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(std::make_unique<CatchIntent>(1, 0, 10, 0.3, 0));

    auto primitive_ptrs = navigator.getAssignedPrimitives(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_ptrs.size(), 1);

    auto expected_primitive = CatchPrimitive(1, 0, 10, 0.3);
    auto primitive          = dynamic_cast<CatchPrimitive &>(*(primitive_ptrs.at(0)));
    EXPECT_EQ(expected_primitive, primitive);
}

TEST_F(ThetaStarNavigatorTest, convert_chip_intent_to_chip_primitive)
{
    World world = ::TestUtil::createBlankTestingWorld();

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(
        std::make_unique<ChipIntent>(0, Point(), Angle::quarter(), 0, 1));

    auto primitive_ptrs = navigator.getAssignedPrimitives(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_ptrs.size(), 1);

    auto expected_primitive = ChipPrimitive(0, Point(), Angle::quarter(), 0);
    auto primitive          = dynamic_cast<ChipPrimitive &>(*(primitive_ptrs.at(0)));
    EXPECT_EQ(expected_primitive, primitive);
}

TEST_F(ThetaStarNavigatorTest,
       convert_direct_velocity_intent_to_direct_velocity_primitive)
{
    World world = ::TestUtil::createBlankTestingWorld();

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(std::make_unique<DirectVelocityIntent>(3, 1, -2, 0.4, 1000, 4));

    auto primitive_ptrs = navigator.getAssignedPrimitives(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_ptrs.size(), 1);

    auto expected_primitive = DirectVelocityPrimitive(3, 1, -2, 0.4, 1000);
    auto primitive = dynamic_cast<DirectVelocityPrimitive &>(*(primitive_ptrs.at(0)));
    EXPECT_EQ(expected_primitive, primitive);
}

TEST_F(ThetaStarNavigatorTest, convert_direct_wheels_intent_to_direct_wheels_primitive)
{
    World world = ::TestUtil::createBlankTestingWorld();

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(
        std::make_unique<DirectWheelsIntent>(2, 80, 22, 55, 201, 5000, 60));

    auto primitive_ptrs = navigator.getAssignedPrimitives(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_ptrs.size(), 1);

    auto expected_primitive = DirectWheelsPrimitive(2, 80, 22, 55, 201, 5000);
    auto primitive = dynamic_cast<DirectWheelsPrimitive &>(*(primitive_ptrs.at(0)));
    EXPECT_EQ(expected_primitive, primitive);
}

TEST_F(ThetaStarNavigatorTest, convert_dribble_intent_to_dribble_primitive)
{
    World world = ::TestUtil::createBlankTestingWorld();

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(
        std::make_unique<DribbleIntent>(0, Point(), Angle::quarter(), 8888, true, 50));

    auto primitive_ptrs = navigator.getAssignedPrimitives(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_ptrs.size(), 1);

    auto expected_primitive = DribblePrimitive(0, Point(), Angle::quarter(), 8888, true);
    auto primitive          = dynamic_cast<DribblePrimitive &>(*(primitive_ptrs.at(0)));
    EXPECT_EQ(expected_primitive, primitive);
}

TEST_F(ThetaStarNavigatorTest, convert_kick_intent_to_kick_primitive)
{
    World world = ::TestUtil::createBlankTestingWorld();

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(
        std::make_unique<KickIntent>(0, Point(), Angle::quarter(), 0, 1));

    auto primitive_ptrs = navigator.getAssignedPrimitives(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_ptrs.size(), 1);

    auto expected_primitive = KickPrimitive(0, Point(), Angle::quarter(), 0);
    auto primitive          = dynamic_cast<KickPrimitive &>(*(primitive_ptrs.at(0)));
    EXPECT_EQ(expected_primitive, primitive);
}

TEST_F(ThetaStarNavigatorTest, convert_movespin_intent_to_movespin_primitive)
{
    World world = ::TestUtil::createBlankTestingWorld();

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(
        std::make_unique<MoveSpinIntent>(0, Point(), AngularVelocity::full(), 1, 0));

    auto primitive_ptrs = navigator.getAssignedPrimitives(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_ptrs.size(), 1);

    auto expected_primitive = MoveSpinPrimitive(0, Point(), AngularVelocity::full(), 1);
    auto primitive          = dynamic_cast<MoveSpinPrimitive &>(*(primitive_ptrs.at(0)));
    EXPECT_EQ(expected_primitive, primitive);
}

TEST_F(ThetaStarNavigatorTest, convert_pivot_intent_to_pivot_primitive)
{
    World world = ::TestUtil::createBlankTestingWorld();

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(std::make_unique<PivotIntent>(0, Point(1, 0.4), Angle::half(),
                                                       Angle::fromRadians(3.2), true, 1));

    auto primitive_ptrs = navigator.getAssignedPrimitives(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_ptrs.size(), 1);

    auto expected_primitive =
        PivotPrimitive(0, Point(1, 0.4), Angle::half(), Angle::fromRadians(3.2), true);
    auto primitive = dynamic_cast<PivotPrimitive &>(*(primitive_ptrs.at(0)));
    EXPECT_EQ(expected_primitive, primitive);
}

TEST_F(ThetaStarNavigatorTest, convert_stop_intent_to_stop_primitive)
{
    World world = ::TestUtil::createBlankTestingWorld();

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(std::make_unique<StopIntent>(0, false, 1));

    auto primitive_ptrs = navigator.getAssignedPrimitives(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_ptrs.size(), 1);

    auto expected_primitive = StopPrimitive(0, false);
    auto primitive          = dynamic_cast<StopPrimitive &>(*(primitive_ptrs.at(0)));
    EXPECT_EQ(expected_primitive, primitive);
}

TEST_F(ThetaStarNavigatorTest, convert_multiple_intents_to_primitives)
{
    World world = ::TestUtil::createBlankTestingWorld();

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(std::make_unique<StopIntent>(0, false, 1));
    intents.emplace_back(std::make_unique<PivotIntent>(0, Point(1, 0.4), Angle::half(),
                                                       Angle::fromRadians(2.2), true, 1));

    auto primitive_ptrs = navigator.getAssignedPrimitives(world, intents);

    // Make sure we got exactly 3 primitives back
    EXPECT_EQ(primitive_ptrs.size(), 2);

    auto expected_stop_primitive = StopPrimitive(0, false);
    auto stop_primitive          = dynamic_cast<StopPrimitive &>(*(primitive_ptrs.at(0)));
    EXPECT_EQ(expected_stop_primitive, stop_primitive);
    auto expected_pivot_primitive =
        PivotPrimitive(0, Point(1, 0.4), Angle::half(), Angle::fromRadians(2.2), true);
    auto pivot_primitive = dynamic_cast<PivotPrimitive &>(*(primitive_ptrs.at(1)));
    EXPECT_EQ(expected_pivot_primitive, pivot_primitive);
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
    Field field = ::TestUtil::createSSLDivBField();

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
                Util::DynamicParameters->getAIConfig()
                    ->getRobotNavigationObstacleFactoryConfig())),
        RobotNavigationObstacleFactory(Util::DynamicParameters->getAIConfig()
                                           ->getRobotNavigationObstacleFactoryConfig()),
        std::make_shared<NavigatorConfig>());

    std::vector<std::unique_ptr<Intent>> intents;
    intents.emplace_back(std::make_unique<MoveIntent>(
        0, poi, Angle::zero(), 0, 0, DribblerEnable::OFF, MoveType::NORMAL,
        AutokickType::NONE, BallCollisionType::AVOID));

    auto primitive_ptrs = navigator.getAssignedPrimitives(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_ptrs.size(), 1);


    auto primitive = dynamic_cast<MovePrimitive &>(*(primitive_ptrs.at(0)));
    EXPECT_EQ(primitive.getDestination(), poi);
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
        AutokickType::NONE, BallCollisionType::AVOID));

    auto primitive_ptrs = navigator.getAssignedPrimitives(world, intents);

    // Make sure we got exactly 1 primitive back
    EXPECT_EQ(primitive_ptrs.size(), 1);


    auto expected_primitive = StopPrimitive(0, false);
    auto primitive          = dynamic_cast<StopPrimitive &>(*(primitive_ptrs.at(0)));
    EXPECT_EQ(expected_primitive, primitive);
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
