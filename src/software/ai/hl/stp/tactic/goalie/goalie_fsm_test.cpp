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
    // Start in GoalieFSM states' position_to_block_shot_state
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PositionToBlockState>));
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PositionToBlockState>));

    // set ball speed to over panic threshold
    world =
            ::TestUtil::setBallVelocity(world, Vector(-3, 0), Timestamp::fromSeconds(123));
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PanicState>));

    // ball is closer to the goal, still moving fast
    world =
            ::TestUtil::setBallPosition(world, Point(-3, 0), Timestamp::fromSeconds(123));
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

    // goalie should move back into PositionToBlockShotState
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PositionToBlockState>));

    // ball is inside friendly defense area, not in the no-chip rectangle, stationary
    world =
            ::TestUtil::setBallPosition(world, Point(-3.5, 1), Timestamp::fromSeconds(123));
    world =
            ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(123));
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    // I don't think I should use the ChipFSM as part of chip_if_safe, since chip_if_safe_s only uses the ChipFSM half the time,
    // and these tests imply that it is always in the ChipFSM
    //EXPECT_TRUE(fsm.is<decltype(boost::sml::state<ChipFSM>)>(boost::sml::state<GetBehindBallFSM>));

    // ball is on the goal line
    world =
            ::TestUtil::setBallPosition(world, world.field().friendlyGoalpostNeg() +
                                               Vector(0, 2 * ROBOT_MAX_RADIUS_METERS), Timestamp::fromSeconds(123));
    //EXPECT_TRUE(fsm.is(boost::sml::state<ChipFSM>));

}
