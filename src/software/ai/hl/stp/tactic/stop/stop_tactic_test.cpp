#include "software/ai/hl/stp/tactic/stop/stop_tactic.h"

#include <gtest/gtest.h>

#include "software/ai/intent/stop_intent.h"
#include "software/test_util/test_util.h"

TEST(StopTacticTest, test_calculate_robot_cost)
{
    World world = ::TestUtil::createBlankTestingWorld();

    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));

    StopTactic tactic = StopTactic(false);

    // We always expect the cost to be 0.5, because the StopTactic prefers all robots
    // equally
    EXPECT_EQ(0.5, tactic.calculateRobotCost(robot, world));
}
