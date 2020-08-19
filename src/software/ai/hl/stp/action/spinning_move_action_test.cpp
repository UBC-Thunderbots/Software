#include "software/ai/hl/stp/action/spinning_move_action.h"

#include <gtest/gtest.h>

#include "software/ai/intent/spinning_move_intent.h"

// TODO (Issue #1644): refactor and reenable these tests
/*
TEST(SpinningMoveActionTest, getDestination)
{
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    SpinningMoveAction action = SpinningMoveAction(0.05);

    action.updateControlParams(robot, Point(7, 13), AngularVelocity::quarter(), 1.0);

    EXPECT_EQ(Point(7, 13), action.getDestination());
}

TEST(SpinningMoveActionTest, robot_far_from_destination)
{
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    SpinningMoveAction action = SpinningMoveAction(0.05);

    action.updateControlParams(robot, Point(1, 0), AngularVelocity::quarter(), 1.0);
    auto intent_ptr = action.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    SpinningMoveIntent spinning_move_intent =
        dynamic_cast<SpinningMoveIntent &>(*intent_ptr);
    EXPECT_EQ(0, spinning_move_intent.getRobotId());
    EXPECT_EQ(Point(1, 0), spinning_move_intent.getDestination());
    EXPECT_EQ(AngularVelocity::quarter(), spinning_move_intent.getAngularVelocity());
    EXPECT_DOUBLE_EQ(1.0, spinning_move_intent.getFinalSpeed());
}

TEST(SpinningMoveActionTest, robot_at_destination)
{
    Robot robot = Robot(0, Point(), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    SpinningMoveAction action = SpinningMoveAction(0.02);

    // We call the action twice. The first time the Intent will always be returned to
    // ensure the Robot is doing the right thing. In all future calls, the action will be
    // done and so will return a null pointer
    action.updateControlParams(robot, Point(0, 0), AngularVelocity::full(), 0);
    action.getNextIntent();
    action.getNextIntent();

    EXPECT_TRUE(action.done());
}

TEST(SpinningMoveActionTest, test_action_does_not_prematurely_report_done)
{
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    SpinningMoveAction action = SpinningMoveAction(0.05);

    // Run the Action several times
    for (int i = 0; i < 10; i++)
    {
        action.updateControlParams(robot, Point(1, 0), AngularVelocity::quarter(), 1.0);
    }
    auto intent_ptr = action.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());
}
*/
