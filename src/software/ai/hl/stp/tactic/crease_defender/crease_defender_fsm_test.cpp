#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_fsm.h"

#include <gtest/gtest.h>

#include "proto/parameters.pb.h"
#include "software/test_util/equal_within_tolerance.h"
#include "software/test_util/test_util.h"

TEST(CreaseDefenderFSMTest, test_find_block_threat_point_in_front_of_crease)
{
    TbotsProto::RobotNavigationObstacleConfig config;
    double robot_obstacle_inflation_factor = config.robot_obstacle_inflation_factor();
    Field field                            = Field::createSSLDivisionBField();
    Point enemy_threat_origin              = Point(2, 3);
    auto threat_point_centre               = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, TbotsProto::CreaseDefenderAlignment::CENTRE,
        robot_obstacle_inflation_factor);
    ASSERT_TRUE(threat_point_centre);
    EXPECT_GT(threat_point_centre.value().x(), field.friendlyDefenseArea().xMax());

    auto threat_point_left = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, TbotsProto::CreaseDefenderAlignment::LEFT,
        robot_obstacle_inflation_factor);
    ASSERT_TRUE(threat_point_left);
    EXPECT_GT(threat_point_left.value().x(), field.friendlyDefenseArea().xMax());

    auto threat_point_right = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, TbotsProto::CreaseDefenderAlignment::RIGHT,
        robot_obstacle_inflation_factor);
    ASSERT_TRUE(threat_point_right);
    EXPECT_GT(threat_point_right.value().x(), field.friendlyDefenseArea().xMax());

    EXPECT_LT(threat_point_centre.value().y(), threat_point_left.value().y());
    EXPECT_GT(threat_point_centre.value().y(), threat_point_right.value().y());
}

TEST(CreaseDefenderFSMTest, test_find_block_threat_point_left_of_crease)
{
    TbotsProto::RobotNavigationObstacleConfig config;
    double robot_obstacle_inflation_factor = config.robot_obstacle_inflation_factor();
    Field field                            = Field::createSSLDivisionBField();
    Point enemy_threat_origin              = Point(-2.5, 3);
    auto threat_point_centre               = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, TbotsProto::CreaseDefenderAlignment::CENTRE,
        robot_obstacle_inflation_factor);
    ASSERT_TRUE(threat_point_centre);
    EXPECT_GE(threat_point_centre.value().y(), field.friendlyDefenseArea().yMax());
    EXPECT_LE(threat_point_centre.value().x(), field.friendlyDefenseArea().xMax());

    auto threat_point_left = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, TbotsProto::CreaseDefenderAlignment::LEFT,
        robot_obstacle_inflation_factor);
    ASSERT_TRUE(threat_point_left);
    EXPECT_GE(threat_point_left.value().y(), field.friendlyDefenseArea().yMax());
    EXPECT_LE(threat_point_left.value().x(), field.friendlyDefenseArea().xMax());

    auto threat_point_right = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, TbotsProto::CreaseDefenderAlignment::RIGHT,
        robot_obstacle_inflation_factor);
    ASSERT_TRUE(threat_point_right);
    EXPECT_GE(threat_point_right.value().y(), field.friendlyDefenseArea().yMax());
    EXPECT_LE(threat_point_right.value().x(), field.friendlyDefenseArea().xMax());

    EXPECT_GT(threat_point_centre.value().x(), threat_point_left.value().x());
    EXPECT_LT(threat_point_centre.value().x(), threat_point_right.value().x());
}

TEST(CreaseDefenderFSMTest, test_find_block_threat_point_right_of_crease)
{
    TbotsProto::RobotNavigationObstacleConfig config;
    double robot_obstacle_inflation_factor = config.robot_obstacle_inflation_factor();
    Field field                            = Field::createSSLDivisionBField();
    Point enemy_threat_origin              = Point(-4.25, -2);
    auto threat_point_centre               = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, TbotsProto::CreaseDefenderAlignment::CENTRE,
        robot_obstacle_inflation_factor);
    ASSERT_TRUE(threat_point_centre);
    EXPECT_LE(threat_point_centre.value().y(), field.friendlyDefenseArea().yMin());
    EXPECT_LE(threat_point_centre.value().x(), field.friendlyDefenseArea().xMax());

    auto threat_point_left = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, TbotsProto::CreaseDefenderAlignment::LEFT,
        robot_obstacle_inflation_factor);
    ASSERT_TRUE(threat_point_left);
    EXPECT_LE(threat_point_left.value().y(), field.friendlyDefenseArea().yMin());
    EXPECT_LE(threat_point_left.value().x(), field.friendlyDefenseArea().xMax());

    auto threat_point_right = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, TbotsProto::CreaseDefenderAlignment::RIGHT,
        robot_obstacle_inflation_factor);
    ASSERT_TRUE(threat_point_right);
    EXPECT_LE(threat_point_right.value().y(), field.friendlyDefenseArea().yMin());
    EXPECT_LE(threat_point_right.value().x(), field.friendlyDefenseArea().xMax());

    EXPECT_LT(threat_point_centre.value().x(), threat_point_left.value().x());
    EXPECT_GT(threat_point_centre.value().x(), threat_point_right.value().x());
}

TEST(CreaseDefenderFSMTest, test_find_block_threat_point_threat_in_crease)
{
    TbotsProto::RobotNavigationObstacleConfig config;
    double robot_obstacle_inflation_factor = config.robot_obstacle_inflation_factor();
    Field field                            = Field::createSSLDivisionBField();
    Point enemy_threat_origin              = Point(-4.25, 0);
    auto threat_point                      = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, TbotsProto::CreaseDefenderAlignment::CENTRE,
        robot_obstacle_inflation_factor);
    EXPECT_FALSE(threat_point);

    threat_point = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, TbotsProto::CreaseDefenderAlignment::LEFT,
        robot_obstacle_inflation_factor);
    EXPECT_FALSE(threat_point);

    threat_point = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, TbotsProto::CreaseDefenderAlignment::RIGHT,
        robot_obstacle_inflation_factor);
    EXPECT_FALSE(threat_point);
}

TEST(CreaseDefenderFSMTest, test_transitions)
{
    TbotsProto::RobotNavigationObstacleConfig config;
    double robot_obstacle_inflation_factor = config.robot_obstacle_inflation_factor();
    World world                            = ::TestUtil::createBlankTestingWorld();
    Robot robot                            = ::TestUtil::createRobotAtPos(Point(-2, -3));
    world =
        ::TestUtil::setBallPosition(world, Point(-0.5, 0), Timestamp::fromSeconds(123));
    CreaseDefenderFSM::ControlParams control_params{
        .enemy_threat_origin       = Point(2, 3),
        .crease_defender_alignment = TbotsProto::CreaseDefenderAlignment::LEFT,
        .max_allowed_speed_mode    = TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT};

    FSM<CreaseDefenderFSM> fsm(CreaseDefenderFSM{config});
    EXPECT_TRUE(fsm.is(boost::sml::state<MoveFSM>));

    // robot far from destination, ball in friendly half
    fsm.process_event(CreaseDefenderFSM::Update(
        control_params, TacticUpdate(
                    robot, world, [](std::unique_ptr<TbotsProto::Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<MoveFSM>));

    auto block_point = CreaseDefenderFSM::findBlockThreatPoint(
        world.field(), control_params.enemy_threat_origin,
        control_params.crease_defender_alignment, robot_obstacle_inflation_factor);

    ASSERT_TRUE(block_point.has_value());
    robot.updateState(
        RobotState(
            block_point.value(), Vector(0, 0),
            (control_params.enemy_threat_origin - block_point.value()).orientation(),
            AngularVelocity::zero()),
        Timestamp::fromSeconds(123));
    // Set robot to the correct position to block the ball
    fsm.process_event(CreaseDefenderFSM::Update(
        control_params, TacticUpdate(
                    robot, world, [](std::unique_ptr<TbotsProto::Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));
    // Check that the FSM stays done
    fsm.process_event(CreaseDefenderFSM::Update(
        control_params, TacticUpdate(
                    robot, world, [](std::unique_ptr<TbotsProto::Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));

    robot.updateState(
        RobotState(
            block_point.value(), Vector(0, 0),
            (control_params.enemy_threat_origin - block_point.value()).orientation() +
                Angle::half(),
            AngularVelocity::zero()),
        Timestamp::fromSeconds(123));
    // change orientation to make the FSM not done
    fsm.process_event(CreaseDefenderFSM::Update(
        control_params, TacticUpdate(
                    robot, world, [](std::unique_ptr<TbotsProto::Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<MoveFSM>));
}
