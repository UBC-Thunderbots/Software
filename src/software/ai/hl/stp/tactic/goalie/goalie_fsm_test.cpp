#include "software/ai/hl/stp/tactic/goalie/goalie_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(GoalieFSMTest, test_transitions)
{
    Robot robot = ::TestUtil::createRobotAtPos(Point(-4, 0));
    World world = ::TestUtil::createBlankTestingWorld();
    world =
            ::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(123));
    world =
            ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(123));
    GoalieFSM::ControlParams control_params{.goalie_tactic_config   = std::make_shared<const GoalieTacticConfig>()};

    HFSM<GoalieFSM> fsm;
    // Start in GoalieFSM states' PositionToBlockShotState
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PositionToBlockState>));

    // ball in motion, away from friendly half
    world =
            ::TestUtil::setBallVelocity(world, Vector(1, 0), Timestamp::fromSeconds(123));
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PositionToBlockState>));

    // ball in motion, towards friendly half
    world =
            ::TestUtil::setBallVelocity(world, Vector(-0.1, 0), Timestamp::fromSeconds(123));
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PositionToBlockState>));

    // set ball speed to over panic threshold
    world =
            ::TestUtil::setBallVelocity(world, Vector(-1, 0), Timestamp::fromSeconds(123));
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PanicState>));

    // goalie has stopped the ball
    world =
            ::TestUtil::setBallPosition(world, Point(-3.8, 0), Timestamp::fromSeconds(123));
    world =
            ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(123));
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));

    // goalie should transition back to PositionToBlockShotState at the next update
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PositionToBlockState>));

    // ball is in the don't-chip rectangle
    world =
            ::TestUtil::setBallPosition(world, world.field().friendlyGoalpostNeg() +
                                               Vector(0, 2 * ROBOT_MAX_RADIUS_METERS), Timestamp::fromSeconds(123));
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::ChipIfSafeState>));

    // ball out of danger, should enter terminal state
    world =
            ::TestUtil::setBallPosition(world, Point(-1, 1), Timestamp::fromSeconds(123));
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));

    // goalie should transition back to PositionToBlockShotState at the next update
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PositionToBlockState>));

    // ball is stationary inside friendly defense area, outside the don't-chip rectangle
    world =
            ::TestUtil::setBallPosition(world, Point(-3.5, 1), Timestamp::fromSeconds(123));
    world =
            ::TestUtil::setBallVelocity(world, Vector(0, -0.1), Timestamp::fromSeconds(123));
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::ChipIfSafeState>));

    // ball starts moving fast, goalie should panic
    world =
            ::TestUtil::setBallPosition(world, Point(-3.5, 1), Timestamp::fromSeconds(123));
    world =
            ::TestUtil::setBallVelocity(world, Vector(-2, -1), Timestamp::fromSeconds(123));
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PanicState>));

    // ball is cleared, goalie should enter terminal state
    world =
            ::TestUtil::setBallPosition(world, Point(2, 0), Timestamp::fromSeconds(123));
    world =
            ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(123));
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));

}
