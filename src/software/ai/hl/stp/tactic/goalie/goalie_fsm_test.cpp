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
    Point clear_ball_origin = Point(-4,0);
    Angle clear_ball_direction = Vector(1,0).orientation();

    GoalieFSM::ControlParams control_params{.goalie_tactic_config   = std::make_shared<const GoalieTacticConfig>(),
            .clear_ball_origin = clear_ball_origin,
            .clear_ball_direction = clear_ball_direction};

    FSM<GoalieFSM> fsm;

    // start in GoalieFSM's PositionToBlockShotState
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PositionToBlockState>));

    // ball is now moving slowly towards the friendly goal
    world =
            ::TestUtil::setBallVelocity(world, Vector(-0.1, 0), Timestamp::fromSeconds(123));
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    // remain in PositionToBlockShotState
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PositionToBlockState>));

    // ball is now moving quickly towards the friendly goal
    world =
            ::TestUtil::setBallVelocity(world, Vector(-1, 0), Timestamp::fromSeconds(123));
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    // transition to PanicState
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PanicState>));

    // ball is out of danger, transition to done
    world = ::TestUtil::setBallVelocity(world, Vector(1,0), Timestamp::fromSeconds(123));
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));

    // process event again to reset to PositionToBlockState
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PositionToBlockState>));

    // ball is now stationary in the "don't-chip" rectangle
    world =
            ::TestUtil::setBallPosition(world, world.field().friendlyGoalpostNeg() +
                                               Vector(ROBOT_MAX_RADIUS_METERS, 2 * ROBOT_MAX_RADIUS_METERS), Timestamp::fromSeconds(123));
    world =
            ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(123));
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    // transition to DribbleFSM
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleFSM>));

    // goalie has ball, at the correct position and orientation to clear the ball
    world =
            ::TestUtil::setBallPosition(world, clear_ball_origin, Timestamp::fromSeconds(123));
    goalie.updateState(RobotState(clear_ball_origin, Vector(),
            clear_ball_direction, AngularVelocity::zero()),Timestamp::fromSeconds(123));

    // process event once to fall through the Dribble FSM
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleFSM>));

    // goalie should now chip the ball
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<ChipFSM>));

    // BROKEN FROM HERE AND BELOW
    world =
            ::TestUtil::setBallVelocity(world, Vector(1,0), Timestamp::fromSeconds(123));

    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<ChipFSM>));

    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));
//    // Ball is now outside of the friendly defense area, away from danger
//    world =
//            ::TestUtil::setBallPosition(world, Point(-1, 1), Timestamp::fromSeconds(123));
//    fsm.process_event(GoalieFSM::Update(
//            control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
//    // Tactic is done
//    EXPECT_TRUE(fsm.is(boost::sml::X));

    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    // Transition back to PositionToBlockState
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PositionToBlockState>));

    // Ball is now moving slowly inside the friendly defense area, outside the "don't-chip" rectangle
    world =
            ::TestUtil::setBallPosition(world, Point(-3.5, 1), Timestamp::fromSeconds(124));
    world =
            ::TestUtil::setBallVelocity(world, Vector(0, -0.1), Timestamp::fromSeconds(124));
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    // Transition to ChipFSM
    EXPECT_TRUE(fsm.is(boost::sml::state<ChipFSM>));

    // Ball is now moving quickly towards the friendly goal
    world =
            ::TestUtil::setBallPosition(world, Point(-3.5, 1), Timestamp::fromSeconds(124));
    world =
            ::TestUtil::setBallVelocity(world, Vector(-2, -1), Timestamp::fromSeconds(124));
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    // Transition to PanicState
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PanicState>));

    // Ball is now out of danger
    world =
            ::TestUtil::setBallPosition(world, Point(2, 0), Timestamp::fromSeconds(124));
    world =
            ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(124));
    fsm.process_event(GoalieFSM::Update(
            control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    // Tactic is done
    EXPECT_TRUE(fsm.is(boost::sml::X));
}
