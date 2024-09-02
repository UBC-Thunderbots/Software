#include "software/ai/hl/stp/tactic/pass_defender/pass_defender_fsm.h"

#include <gtest/gtest.h>

#include "proto/parameters.pb.h"
#include "software/test_util/test_util.h"

TEST(PassDefenderFSMTest, test_transitions)
{
    std::shared_ptr<World> world    = ::TestUtil::createBlankTestingWorld();
    Robot robot                     = ::TestUtil::createRobotAtPos(Point(-1, 0));
    std::vector<Point> enemy_robots = {
        Point(1, 0),
    };
    ::TestUtil::setEnemyRobotPositions(world, enemy_robots, Timestamp::fromSeconds(123));
    PassDefenderFSM::ControlParams control_params{
        .position_to_block_from = Point(-2, 0),
        .ball_steal_mode        = TbotsProto::BallStealMode::STEAL};
    TbotsProto::AiConfig ai_config;
    FSM<PassDefenderFSM> fsm{PassDefenderFSM(ai_config),
                             DribbleFSM(ai_config.dribble_tactic_config())};

    // Start in BlockPassState
    EXPECT_TRUE(fsm.is(boost::sml::state<PassDefenderFSM::BlockPassState>));

    // Ball has not been kicked towards the pass defender (i.e. enemy has possession)
    // Should stay in BlockPassState
    fsm.process_event(PassDefenderFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<PassDefenderFSM::BlockPassState>));

    // Ball is now kicked towards pass defender
    ::TestUtil::setBallVelocity(world, Vector(-1, 0), Timestamp::fromSeconds(123));
    EXPECT_TRUE(world->ball().hasBallBeenKicked(Angle::half()));

    // Transition to InterceptBallState
    fsm.process_event(PassDefenderFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<PassDefenderFSM::InterceptBallState>));

    // Deflect ball away from pass defender and return to block state
    ::TestUtil::setBallPosition(world, Point(-0.5, 0), Timestamp::fromSeconds(124));
    ::TestUtil::setBallVelocity(world, Vector(0, 1), Timestamp::fromSeconds(125));
    EXPECT_TRUE(world->ball().hasBallBeenKicked(Angle::quarter()));
    fsm.process_event(PassDefenderFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<PassDefenderFSM::BlockPassState>));

    // Ball is now again kicked towards pass defender
    ::TestUtil::setBallVelocity(world, Vector(-1, 0), Timestamp::fromSeconds(125));
    EXPECT_TRUE(world->ball().hasBallBeenKicked(Angle::half()));

    // Transition to InterceptBallState again
    fsm.process_event(PassDefenderFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<PassDefenderFSM::InterceptBallState>));

    // Move Ball Close and Transition to DribbleFSM from interception
    ::TestUtil::setBallVelocity(world, Vector(-0.1, 0), Timestamp::fromSeconds(126));
    ::TestUtil::setBallPosition(world, Point(-1.8, 0), Timestamp::fromSeconds(126));
    fsm.process_event(PassDefenderFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleFSM>));

    ::TestUtil::setBallPosition(world, Point(-0.5, 0), Timestamp::fromSeconds(126));
    ::TestUtil::setBallVelocity(world, Vector(0, 1), Timestamp::fromSeconds(126));
    // Deflect and Transition back to BlockPassState
    fsm.process_event(PassDefenderFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<PassDefenderFSM::BlockPassState>));
}

// This is created to test one single edge case in interceptBall
TEST(PassDefenderFSMTest, test_intercept_edge_case)
{
    // create the world and the robot to test at (0,0)
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    Robot robot                  = ::TestUtil::createRobotAtPos(Point(0, 0));
    PassDefenderFSM::ControlParams control_params{
        .position_to_block_from = Point(-2, 0),
        .ball_steal_mode        = TbotsProto::BallStealMode::STEAL};
    TbotsProto::AiConfig ai_config;

    FSM<PassDefenderFSM> fsm{PassDefenderFSM(ai_config),
                             DribbleFSM(ai_config.dribble_tactic_config())};

    // Start in BlockPassState
    EXPECT_TRUE(fsm.is(boost::sml::state<PassDefenderFSM::BlockPassState>));

    // set ball to be inside of robot at (0,0)
    TestUtil::setBallPosition(world, Point(0, 0), Timestamp());

    // Ball is now kicked "towards" pass defender
    ::TestUtil::setBallVelocity(world, Vector(-2, 0), Timestamp::fromSeconds(123));
    EXPECT_TRUE(world->ball().hasBallBeenKicked(Angle::half()));

    std::shared_ptr<Primitive> primitive;
    fsm.process_event(PassDefenderFSM::Update(
        control_params,
        TacticUpdate(robot, world,
                     [&primitive](std::shared_ptr<Primitive> x) { primitive = x; })));
    EXPECT_TRUE(primitive != nullptr);
    EXPECT_TRUE(fsm.is(boost::sml::state<PassDefenderFSM::InterceptBallState>));
}
