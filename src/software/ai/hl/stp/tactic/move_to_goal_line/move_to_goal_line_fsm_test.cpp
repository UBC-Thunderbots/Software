#include "software/ai/hl/stp/tactic/move_to_goal_line/move_to_goal_line_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(MoveToGoalLineFSMTest, test_transitions)
{
    World world = ::TestUtil::createBlankTestingWorld();
    Robot robot(0,
                RobotState(Point(0, 0), Vector(0, 0), Angle::half(),
                           AngularVelocity::zero()),
                Timestamp::fromSeconds(123));

    FSM<MoveToGoalLineFSM> fsm;
    EXPECT_TRUE(fsm.is(boost::sml::state<MoveFSM::MoveState>));
    fsm.process_event();
}
