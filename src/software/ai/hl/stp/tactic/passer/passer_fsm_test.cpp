#include "software/ai/hl/stp/tactic/passer/passer_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(PasserFSMTest, test_transitions)
{
    World world = ::TestUtil::createBlankTestingWorld();
    Robot robot = ::TestUtil::createRobotAtPos(Point(-2, -3));
    Pass pass   = Pass(Point(0, 0), Point(2, 0), 5, Timestamp::fromSeconds(0));

    PasserFSM::ControlParams control_params{.pass = std::make_optional<Pass>(pass)};

    FSM<PasserFSM> fsm;
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleFSM>));

    // robot far from passer point
    fsm.process_event(PasserFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleFSM>));

    // robot close to passer point
    robot = ::TestUtil::createRobotAtPos(Point(2, 2));
    fsm.process_event(PasserFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleFSM>));

    // robot at passer point and facing the right way
    robot.updateState(RobotState(pass.passerPoint(), Vector(), pass.passerOrientation(),
                                 AngularVelocity::zero()),
                      Timestamp::fromSeconds(0));

    // process event once to fall through the Dribble FSM
    fsm.process_event(PasserFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleFSM>));

    // robot should now kick the ball
    fsm.process_event(PasserFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<KickFSM>));
}
