#include <gtest/gtest.h>

#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"
#include "software/test_util/test_util.h"

TEST(FsmStateTest, test_get_fsm_state)
{
    AttackerTactic tactic(std::make_shared<AttackerTacticConfig>());

    World world = ::TestUtil::createBlankTestingWorld();
    Robot robot = ::TestUtil::createRobotAtPos(Point(-2, -3));
    Pass pass   = Pass(Point(0, 0), Point(2, 0), 5);

    EXPECT_EQ("DribbleFSM.GetPossessionState", tactic.getFSMState());
    tactic.updateControlParams(pass, true);
    tactic.get(robot, world);

    // robot at attacker point and facing the right way
    robot.updateState(RobotState(pass.passerPoint() - Vector(0.05, 0), Vector(),
                                 pass.passerOrientation(), AngularVelocity::zero()),
                      Timestamp::fromSeconds(0));

    // process event once to fall through the Dribble FSM
    tactic.get(robot, world);
    EXPECT_EQ("PivotKickFSM.DribbleFSM.DribbleState", tactic.getFSMState());

    // robot should now kick the ball
    tactic.get(robot, world);
    EXPECT_EQ("PivotKickFSM.KickState", tactic.getFSMState());

    // FSM should be done now
    world = ::TestUtil::setBallVelocity(world, Vector(5, 0), Timestamp::fromSeconds(223));
    EXPECT_TRUE(world.ball().hasBallBeenKicked(pass.passerOrientation()));
    tactic.get(robot, world);
    EXPECT_EQ("terminate_state", tactic.getFSMState());
}
