#include "software/ai/hl/stp/tactic/penalty_kick_tactic/penalty_kick_tactic_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(PenaltyKickTacticFSM, test_transitions) {
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world, world.field().friendlyPenaltyMark(), Timestamp::fromSeconds(0));
    Robot robot = ::TestUtil::createRobotAtPos(Point(-2, 0));
    
    HFSM<PenaltyKickTacticFSM> fsm;
    EXPECT_TRUE(fsm.is(boost::sml::state<PenaltyKickTacticFSM::InitialState>));

    PenaltyKickTacticFSM::ControlParams control_params{
        .enemy_goalie = std::nullopt
    };

    fsm.process_event(PenaltyKickTacticFSM::Update(
        control_params,
        TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallFSM>));

    robot = ::TestUtil::createRobotAtPos(world.field().friendlyPenaltyMark());
    fsm.process_event(PenaltyKickTacticFSM::Update(
        control_params,
        TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleFSM>));

    robot = ::TestUtil::createRobotAtPos(world.field().enemyGoalCenter()
        + Vector(-1, 0));
    fsm.process_event(PenaltyKickTacticFSM::Update(
        control_params,
        TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<KickFSM>));

    fsm.process_event(PenaltyKickTacticFSM::Update(
        control_params,
        TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));
}