#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/equal_within_tolerance.h"
#include "software/test_util/test_util.h"

TEST(CreaseDefenderFSMTest, test_find_block_threat_point_in_front_of_crease)
{
    Field field               = Field::createSSLDivisionBField();
    Point enemy_threat_origin = Point(2, 3);
    auto threat_point         = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, CreaseDefenderAlignment::CENTRE);
    ASSERT_TRUE(threat_point);
    EXPECT_TRUE(
        TestUtil::equalWithinTolerance(*threat_point, Point(-3.365, 0.536), 1e-3));

    threat_point = CreaseDefenderFSM::findBlockThreatPoint(field, enemy_threat_origin,
                                                           CreaseDefenderAlignment::LEFT);
    ASSERT_TRUE(threat_point);
    EXPECT_TRUE(
        TestUtil::equalWithinTolerance(*threat_point, Point(-3.365, 0.805), 1e-3));

    threat_point = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, CreaseDefenderAlignment::RIGHT);
    ASSERT_TRUE(threat_point);
    EXPECT_TRUE(
        TestUtil::equalWithinTolerance(*threat_point, Point(-3.365, 0.256), 1e-3));
}

TEST(CreaseDefenderFSMTest, test_find_block_threat_point_left_of_crease)
{
    Field field               = Field::createSSLDivisionBField();
    Point enemy_threat_origin = Point(-2.25, 3);
    auto threat_point         = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, CreaseDefenderAlignment::CENTRE);
    ASSERT_TRUE(threat_point);
    EXPECT_TRUE(
        TestUtil::equalWithinTolerance(*threat_point, Point(-3.674, 1.135), 1e-3));

    threat_point = CreaseDefenderFSM::findBlockThreatPoint(field, enemy_threat_origin,
                                                           CreaseDefenderAlignment::LEFT);
    ASSERT_TRUE(threat_point);
    EXPECT_TRUE(
        TestUtil::equalWithinTolerance(*threat_point, Point(-3.840, 1.135), 1e-3));

    threat_point = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, CreaseDefenderAlignment::RIGHT);
    ASSERT_TRUE(threat_point);
    EXPECT_TRUE(
        TestUtil::equalWithinTolerance(*threat_point, Point(-3.521, 1.135), 1e-3));
}

TEST(CreaseDefenderFSMTest, test_find_block_threat_point_right_of_crease)
{
    Field field               = Field::createSSLDivisionBField();
    Point enemy_threat_origin = Point(-4.25, -2);
    auto threat_point         = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, CreaseDefenderAlignment::CENTRE);
    ASSERT_TRUE(threat_point);
    EXPECT_TRUE(
        TestUtil::equalWithinTolerance(*threat_point, Point(-4.365, -1.135), 1e-3));

    threat_point = CreaseDefenderFSM::findBlockThreatPoint(field, enemy_threat_origin,
                                                           CreaseDefenderAlignment::LEFT);
    ASSERT_TRUE(threat_point);
    EXPECT_TRUE(
        TestUtil::equalWithinTolerance(*threat_point, Point(-4.346, -1.135), 1e-3));

    threat_point = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, CreaseDefenderAlignment::RIGHT);
    ASSERT_TRUE(threat_point);
    EXPECT_TRUE(
        TestUtil::equalWithinTolerance(*threat_point, Point(-4.384, -1.135), 1e-3));
}

TEST(CreaseDefenderFSMTest, test_find_block_threat_point_threat_in_crease)
{
    Field field               = Field::createSSLDivisionBField();
    Point enemy_threat_origin = Point(-4.25, 0);
    auto threat_point         = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, CreaseDefenderAlignment::CENTRE);
    EXPECT_FALSE(threat_point);

    threat_point = CreaseDefenderFSM::findBlockThreatPoint(field, enemy_threat_origin,
                                                           CreaseDefenderAlignment::LEFT);
    EXPECT_FALSE(threat_point);

    threat_point = CreaseDefenderFSM::findBlockThreatPoint(
        field, enemy_threat_origin, CreaseDefenderAlignment::RIGHT);
    EXPECT_FALSE(threat_point);
}

TEST(CreaseDefenderFSMTest, test_transitions)
{
    World world = ::TestUtil::createBlankTestingWorld();
    Robot robot = ::TestUtil::createRobotAtPos(Point(-2, -3));
    world =
        ::TestUtil::setBallPosition(world, Point(-0.5, 0), Timestamp::fromSeconds(123));
    CreaseDefenderFSM::ControlParams control_params{
        .enemy_threat_origin       = Point(2, 3),
        .crease_defender_alignment = CreaseDefenderAlignment::LEFT};

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
