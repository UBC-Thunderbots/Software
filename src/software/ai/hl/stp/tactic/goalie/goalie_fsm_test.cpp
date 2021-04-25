#include "software/ai/hl/stp/tactic/goalie/goalie_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(GoalieFSMTest, test_transitions)
{
    Robot goalie = ::TestUtil::createRobotAtPos(Point(-4.5, 0));
    World world  = ::TestUtil::createBlankTestingWorld();

    world = ::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(123));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(123));
    Point clear_ball_origin    = world.field().friendlyGoalCenter() + Vector(0.5, 0);
    Angle clear_ball_direction = Angle::zero();

    GoalieFSM::ControlParams control_params{
        .goalie_tactic_config = std::make_shared<const GoalieTacticConfig>(),
        .clear_ball_origin    = clear_ball_origin,
        .clear_ball_direction = clear_ball_direction};

    FSM<GoalieFSM> fsm;

    // goalie starts in PositionToBlockState
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PositionToBlockState>));

    // ball is now moving slowly towards the friendly goal
    world =
        ::TestUtil::setBallVelocity(world, Vector(-0.1, 0), Timestamp::fromSeconds(123));

    // goalie should remain in PositionToBlockState
    fsm.process_event(GoalieFSM::Update(
        control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PositionToBlockState>));

    // ball is now moving quickly towards the friendly goal
    world =
        ::TestUtil::setBallVelocity(world, Vector(-1, 0), Timestamp::fromSeconds(123));

    // goalie should transition to PanicState
    fsm.process_event(GoalieFSM::Update(
        control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PanicState>));

    // ball is now out of danger
    world = ::TestUtil::setBallVelocity(world, Vector(1, 0), Timestamp::fromSeconds(123));

    // tactic is done
    fsm.process_event(GoalieFSM::Update(
        control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));

    // process event again to reset goalie to PositionToBlockState
    fsm.process_event(GoalieFSM::Update(
        control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PositionToBlockState>));

    // ball is now stationary in the "don't-chip" rectangle
    world = ::TestUtil::setBallPosition(
        world,
        world.field().friendlyGoalpostNeg() +
            Vector(ROBOT_MAX_RADIUS_METERS, 2 * ROBOT_MAX_RADIUS_METERS),
        Timestamp::fromSeconds(123));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(123));

    // goalie should transition to DribbleFSM
    fsm.process_event(GoalieFSM::Update(
        control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleFSM>));

    // goalie has ball, at the correct position and orientation to clear the ball
    world = ::TestUtil::setBallPosition(world, clear_ball_origin,
                                        Timestamp::fromSeconds(123));
    goalie.updateState(RobotState(clear_ball_origin, Vector(0, 0), clear_ball_direction,
                                  AngularVelocity::zero()),
                       Timestamp::fromSeconds(123));

    // process event once to fall through the DribbleFSM
    fsm.process_event(GoalieFSM::Update(
        control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleFSM>));

    // goalie should transition to ChipFSM
    fsm.process_event(GoalieFSM::Update(
        control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<ChipFSM>));

    // ball is now chipped
    goalie = ::TestUtil::createRobotAtPos(clear_ball_origin + Vector(-0.2, 0));
    world  = ::TestUtil::setBallPosition(world, clear_ball_origin,
                                        Timestamp::fromSeconds(123));
    world = ::TestUtil::setBallVelocity(world, Vector(1, 0), Timestamp::fromSeconds(123));
    EXPECT_TRUE(world.ball().hasBallBeenKicked(clear_ball_direction));

    // process event once to fall through the ChipFSM
    fsm.process_event(GoalieFSM::Update(
        control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<ChipFSM>));

    // tactic is done
    fsm.process_event(GoalieFSM::Update(
        control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));

    // process event once to reset goalie to PositionToBlockState
    fsm.process_event(GoalieFSM::Update(
        control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PositionToBlockState>));

    // ball is now moving slowly inside the friendly defense area
    world =
        ::TestUtil::setBallPosition(world, Point(-3.5, 1), Timestamp::fromSeconds(124));
    world =
        ::TestUtil::setBallVelocity(world, Vector(0, -0.1), Timestamp::fromSeconds(124));

    // goalie should transition to ChipFSM
    fsm.process_event(GoalieFSM::Update(
        control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<ChipFSM>));

    // ball is now moving quickly towards the friendly goal
    world =
        ::TestUtil::setBallPosition(world, Point(-3.5, 1), Timestamp::fromSeconds(124));
    world =
        ::TestUtil::setBallVelocity(world, Vector(-2, -1), Timestamp::fromSeconds(124));

    // goalie should transition to PanicState
    fsm.process_event(GoalieFSM::Update(
        control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PanicState>));

    // ball is now out of danger
    world = ::TestUtil::setBallPosition(world, Point(2, 0), Timestamp::fromSeconds(124));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(124));
    fsm.process_event(GoalieFSM::Update(
        control_params, TacticUpdate(goalie, world, [](std::unique_ptr<Intent>) {})));
    // tactic is done
    EXPECT_TRUE(fsm.is(boost::sml::X));
}
