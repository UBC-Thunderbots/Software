#include "ai/hl/stp/action/move_action.h"

#include <gtest/gtest.h>

#include <boost/coroutine2/all.hpp>

#include "ai/intent/move_intent.h"

TEST(MoveActionTest, robot_far_from_destination)
{
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    MoveAction action = MoveAction(robot);

    auto intent_ptr =
        action.updateStateAndGetNextIntent(robot, Point(1, 0), Angle::quarter(), 1.0);

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);

    MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
    EXPECT_EQ(0, move_intent.getRobotId());
    EXPECT_EQ(Point(1, 0), move_intent.getDestination());
    EXPECT_EQ(Angle::quarter(), move_intent.getFinalAngle());
    EXPECT_EQ(1.0, move_intent.getFinalSpeed());
}

TEST(MoveActionTest, robot_at_destination)
{
    Robot robot = Robot(0, Point(), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    MoveAction action = MoveAction(robot);

    // We call the action twice. The first time the Intent will always be returned to
    // ensure the Robot is doing the right thing. In all future calls, the action will be
    // done and so will return a null pointer
    auto intent_ptr =
        action.updateStateAndGetNextIntent(robot, Point(0, 0), Angle::zero(), 0.0);
    intent_ptr =
        action.updateStateAndGetNextIntent(robot, Point(0, 0), Angle::zero(), 0.0);

    // We expect the resultant pointer to be null since the Action is done and has no more
    // Intents to return, since the robot is already at the destination
    EXPECT_FALSE(intent_ptr);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
