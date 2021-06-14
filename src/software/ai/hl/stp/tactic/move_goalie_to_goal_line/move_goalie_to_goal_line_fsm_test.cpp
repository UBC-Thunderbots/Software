#include "software/ai/hl/stp/tactic/move_goalie_to_goal_line/move_goalie_to_goal_line_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(MoveGoalieToGoalLineFSMTest, test_transitions)
{
    World world = ::TestUtil::createBlankTestingWorld();
    Robot robot(
        0, RobotState(Point(0, 0), Vector(0, 0), Angle::half(), AngularVelocity::zero()),
        Timestamp::fromSeconds(123));

    MoveGoalieToGoalLineFSM::ControlParams control_params{};

    FSM<MoveGoalieToGoalLineFSM> fsm;
    EXPECT_TRUE(fsm.is(boost::sml::state<MoveFSM>));

    // robot far from goal center
    fsm.process_event(MoveGoalieToGoalLineFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<MoveFSM>));

    // robot close to goal center
    robot =
        ::TestUtil::createRobotAtPos(Point(world.field().friendlyGoalCenter().x() + 0.5,
                                           world.field().friendlyGoalCenter().y() + 0.5));
    fsm.process_event(MoveGoalieToGoalLineFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<MoveFSM>));

    // robot at goal center and facing the right way
    robot.updateState(RobotState(world.field().friendlyGoalCenter(), Vector(0, 0),
                                 Angle::zero(), AngularVelocity::zero()),
                      Timestamp::fromSeconds(0));
    fsm.process_event(MoveGoalieToGoalLineFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));
}
