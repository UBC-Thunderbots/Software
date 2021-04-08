#include "software/ai/hl/stp/tactic/goalie/goalie_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(GoalieFSMTest, test_transitions)
{
    Robot goalie = ::TestUtil::createRobotAtPos(Point(-4.5, 0));
    World world = ::TestUtil::createBlankTestingWorld();
    world =
            ::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(123));
    world =
            ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(123));
    GoalieFSM::ControlParams control_params{.goalie_tactic_config   = std::make_shared<const GoalieTacticConfig>()};

    FSM<GoalieFSM> fsm;

    // Start in GoalieFSM's PositionToBlockShotState
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PositionToBlockState>));

    // Ball is now moving away from the friendly goal
    world =
            ::TestUtil::setBallVelocity(world, Vector(1, 0), Timestamp::fromSeconds(123));
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    // GoalieFSM should remain in PositionToBlockShotState
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PositionToBlockState>));

    // Ball is now moving slowly towards the friendly goal
    world =
            ::TestUtil::setBallVelocity(world, Vector(-0.1, 0), Timestamp::fromSeconds(123));
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    // Remain in PositionToBlockShotState
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PositionToBlockState>));

    // Ball is now moving quickly towards the friendly goal
    world =
            ::TestUtil::setBallVelocity(world, Vector(-1, 0), Timestamp::fromSeconds(123));
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    // Transition to PanicState
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PanicState>));

    // Ball is now stationary in the "don't-chip" rectangle
    world =
            ::TestUtil::setBallPosition(world, world.field().friendlyGoalpostNeg() +
                                               Vector(ROBOT_MAX_RADIUS_METERS, 2 * ROBOT_MAX_RADIUS_METERS), Timestamp::fromSeconds(123));
    world =
            ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(123));
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    // Transition to DribbleFSM's get possession state
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<DribbleFSM>)>(
            boost::sml::state<DribbleFSM::GetPossessionState>));

    // Ball is now outside of the "don't chip" rectangle, in the friendly defense area,
    // so should transition to ChipFSM
    world =
            ::TestUtil::setBallPosition(world, Point(-3.5, 1), Timestamp::fromSeconds(123));
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<ChipFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<ChipFSM>)>(
            boost::sml::state<GetBehindBallFSM>));

    // Ball is now outside of the friendly defense area, away from danger
    world =
            ::TestUtil::setBallPosition(world, Point(-1, 1), Timestamp::fromSeconds(123));
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    // Tactic is done
    EXPECT_TRUE(fsm.is(boost::sml::X));
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    // Transition back to PositionToBlockState
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PositionToBlockState>));

    // Ball is now moving slowly inside the friendly defense area, outside the "don't-chip" rectangle
    world =
            ::TestUtil::setBallPosition(world, Point(-3.5, 1), Timestamp::fromSeconds(123));
    world =
            ::TestUtil::setBallVelocity(world, Vector(0, -0.1), Timestamp::fromSeconds(123));
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    // Transition to ChipFSM
    EXPECT_TRUE(fsm.is(boost::sml::state<ChipFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<ChipFSM>)>(
            boost::sml::state<GetBehindBallFSM>));

    // Ball is now moving quickly towards the friendly goal
    world =
            ::TestUtil::setBallPosition(world, Point(-3.5, 1), Timestamp::fromSeconds(123));
    world =
            ::TestUtil::setBallVelocity(world, Vector(-2, -1), Timestamp::fromSeconds(123));
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    // Transition to PanicState
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PanicState>));

    // Ball is now out of danger
    world =
            ::TestUtil::setBallPosition(world, Point(2, 0), Timestamp::fromSeconds(123));
    world =
            ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(123));
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    // Tactic is done
    EXPECT_TRUE(fsm.is(boost::sml::X));
}
