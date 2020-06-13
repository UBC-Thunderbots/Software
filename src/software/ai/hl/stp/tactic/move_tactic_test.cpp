#include "software/ai/hl/stp/tactic/move_tactic.h"

#include <gtest/gtest.h>

#include "software/ai/hl/stp/action/move_action.h"
#include "software/test_util/test_util.h"

TEST(MoveTacticTest, robot_far_from_destination)
{
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));

    MoveTactic tactic = MoveTactic(false);
    tactic.updateRobot(robot);
    tactic.updateControlParams(Point(1, 0), Angle::quarter(), 1.0);
    auto action_ptr = tactic.getNextAction();

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);

    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(move_action, nullptr);
    ASSERT_TRUE(move_action->getRobot().has_value());
    EXPECT_EQ(0, move_action->getRobot()->id());
    EXPECT_EQ(Point(1, 0), move_action->getDestination());
    EXPECT_EQ(Angle::quarter(), move_action->getFinalOrientation());
    EXPECT_EQ(1.0, move_action->getFinalSpeed());
}

TEST(MoveTacticTest, robot_at_destination)
{
    Robot robot = Robot(0, Point(), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));

    MoveTactic tactic = MoveTactic(false);
    tactic.updateRobot(robot);
    tactic.updateControlParams(Point(0, 0), Angle::zero(), 0.0);

    auto action_ptr = tactic.getNextAction();
    ASSERT_NE(action_ptr, nullptr);

    // We call the Action twice. The first time the Intent will always be returned to
    // ensure the Robot is doing the right thing. In all future calls, the action will be
    // done and so will return a null pointer
    EXPECT_NE(nullptr, action_ptr->getNextIntent());
    EXPECT_EQ(nullptr, action_ptr->getNextIntent());

    action_ptr = tactic.getNextAction();
    EXPECT_EQ(nullptr, tactic.getNextAction());
    EXPECT_TRUE(tactic.done());
}

TEST(MoveTacticTest, test_calculate_robot_cost)
{
    World world = ::TestUtil::createBlankTestingWorld();

    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));

    MoveTactic tactic = MoveTactic(false);
    tactic.updateControlParams(Point(3, -4), Angle::zero(), 0.0);

    EXPECT_EQ(5 / world.field().totalXLength(), tactic.calculateRobotCost(robot, world));
}
