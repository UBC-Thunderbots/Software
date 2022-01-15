#include "software/ai/hl/stp/tactic/move/move_tactic.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(MoveTacticTest, test_calculate_robot_cost)
{
    World world = ::TestUtil::createBlankTestingWorld();

    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));

    MoveTactic tactic = MoveTactic(false);
    tactic.updateControlParams(Point(3, -4), Angle::zero(), 0.0);

    EXPECT_EQ(5 / world.field().totalXLength(), tactic.calculateRobotCost(robot, world));
}
