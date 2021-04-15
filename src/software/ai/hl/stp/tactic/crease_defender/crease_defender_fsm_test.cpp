#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_fsm.h"

#include <gtest/gtest.h>

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/test_util/equal_within_tolerance.h"
#include "software/test_util/test_util.h"

TEST(CreaseDefenderFSMTest, test_find_block_threat_point_in_front_of_crease)
{
    double robot_obstacle_inflation_factor = 2.0167;
    Field field                            = Field::createSSLDivisionBField();
    Point enemy_threat_origin              = Point(2, 3);
    auto threat_point                      = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, CreaseDefenderAlignment::CENTRE,
        robot_obstacle_inflation_factor);
    ASSERT_TRUE(threat_point);
    EXPECT_TRUE(
        TestUtil::equalWithinTolerance(threat_point.value(), Point(-3.318, 0.557), 1e-3));

    threat_point = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, CreaseDefenderAlignment::LEFT,
        robot_obstacle_inflation_factor);
    ASSERT_TRUE(threat_point);
    EXPECT_TRUE(
        TestUtil::equalWithinTolerance(threat_point.value(), Point(-3.318, 0.824), 1e-3));

    threat_point = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, CreaseDefenderAlignment::RIGHT,
        robot_obstacle_inflation_factor);
    ASSERT_TRUE(threat_point);
    EXPECT_TRUE(
        TestUtil::equalWithinTolerance(threat_point.value(), Point(-3.318, 0.280), 1e-3));
}

TEST(CreaseDefenderFSMTest, test_find_block_threat_point_left_of_crease)
{
    double robot_obstacle_inflation_factor = 2.0167;
    Field field                            = Field::createSSLDivisionBField();
    Point enemy_threat_origin              = Point(-2.25, 3);
    auto threat_point                      = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, CreaseDefenderAlignment::CENTRE,
        robot_obstacle_inflation_factor);
    ASSERT_TRUE(threat_point);
    EXPECT_TRUE(
        TestUtil::equalWithinTolerance(threat_point.value(), Point(-3.638, 1.182), 1e-3));

    threat_point = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, CreaseDefenderAlignment::LEFT,
        robot_obstacle_inflation_factor);
    ASSERT_TRUE(threat_point);
    EXPECT_TRUE(
        TestUtil::equalWithinTolerance(threat_point.value(), Point(-3.8, 1.182), 1e-3));

    threat_point = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, CreaseDefenderAlignment::RIGHT,
        robot_obstacle_inflation_factor);
    ASSERT_TRUE(threat_point);
    EXPECT_TRUE(
        TestUtil::equalWithinTolerance(threat_point.value(), Point(-3.489, 1.182), 1e-3));
}

TEST(CreaseDefenderFSMTest, test_find_block_threat_point_right_of_crease)
{
    double robot_obstacle_inflation_factor = 2.0167;
    Field field                            = Field::createSSLDivisionBField();
    Point enemy_threat_origin              = Point(-4.25, -2);
    auto threat_point                      = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, CreaseDefenderAlignment::CENTRE,
        robot_obstacle_inflation_factor);
    ASSERT_TRUE(threat_point);
    EXPECT_TRUE(TestUtil::equalWithinTolerance(threat_point.value(),
                                               Point(-4.359, -1.182), 1e-3));

    threat_point = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, CreaseDefenderAlignment::LEFT,
        robot_obstacle_inflation_factor);
    ASSERT_TRUE(threat_point);
    EXPECT_TRUE(TestUtil::equalWithinTolerance(threat_point.value(),
                                               Point(-4.341, -1.182), 1e-3));

    threat_point = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, CreaseDefenderAlignment::RIGHT,
        robot_obstacle_inflation_factor);
    ASSERT_TRUE(threat_point);
    EXPECT_TRUE(TestUtil::equalWithinTolerance(threat_point.value(),
                                               Point(-4.377, -1.182), 1e-3));
}

TEST(CreaseDefenderFSMTest, test_find_block_threat_point_threat_in_crease)
{
    double robot_obstacle_inflation_factor = 2.0167;
    Field field                            = Field::createSSLDivisionBField();
    Point enemy_threat_origin              = Point(-4.25, 0);
    auto threat_point                      = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, CreaseDefenderAlignment::CENTRE,
        robot_obstacle_inflation_factor);
    EXPECT_FALSE(threat_point);

    threat_point = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, CreaseDefenderAlignment::LEFT,
        robot_obstacle_inflation_factor);
    EXPECT_FALSE(threat_point);

    threat_point = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, CreaseDefenderAlignment::RIGHT,
        robot_obstacle_inflation_factor);
    EXPECT_FALSE(threat_point);
}

TEST(CreaseDefenderFSMTest, test_transitions)
{
    double robot_obstacle_inflation_factor = 2.0167;
    World world                            = ::TestUtil::createBlankTestingWorld();
    Robot robot                            = ::TestUtil::createRobotAtPos(Point(-2, -3));
    world =
        ::TestUtil::setBallPosition(world, Point(-0.5, 0), Timestamp::fromSeconds(123));
    CreaseDefenderFSM::ControlParams control_params{
        .enemy_threat_origin             = Point(2, 3),
        .crease_defender_alignment       = CreaseDefenderAlignment::LEFT,
        .robot_obstacle_inflation_factor = robot_obstacle_inflation_factor};

    FSM<CreaseDefenderFSM> fsm;
    EXPECT_TRUE(fsm.is(boost::sml::state<CreaseDefenderFSM::BlockThreatState>));

    // robot far from destination, ball in friendly half
    fsm.process_event(CreaseDefenderFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<CreaseDefenderFSM::BlockThreatState>));

    // robot far from destination, ball in enemy half
    world =
        ::TestUtil::setBallPosition(world, Point(2.5, 0), Timestamp::fromSeconds(123));
    fsm.process_event(CreaseDefenderFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));
}
