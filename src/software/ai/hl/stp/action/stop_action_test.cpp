#include "software/ai/hl/stp/action/stop_action.h"

#include <gtest/gtest.h>

#include "software/ai/intent/stop_intent.h"


TEST(StopActionTest, robot_stopping_without_coasting_while_already_moving)
{
    Robot robot       = Robot(0, Point(10, 10), Vector(1, 3), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    StopAction action = StopAction(0.05, false);

    auto intent_ptr = action.updateStateAndGetNextIntent(robot, false);

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    try
    {
        StopIntent stop_intent = dynamic_cast<StopIntent &>(*intent_ptr);
        EXPECT_EQ(0, stop_intent.getRobotId());
    }
    catch (...)
    {
        ADD_FAILURE() << "StopIntent was not returned by the StopAction!";
    }
}

TEST(StopAction, robot_stopping_while_already_stopped)
{
    Robot robot       = Robot(0, Point(10, 10), Vector(), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    StopAction action = StopAction(0.05, false);

    // We call the action twice. The first time the Intent will always be returned to
    // ensure the Robot is doing the right thing. In all future calls, the action will be
    // done and so will return a null pointer
    auto intent_ptr = action.updateStateAndGetNextIntent(robot, false);
    intent_ptr      = action.updateStateAndGetNextIntent(robot, false);

    EXPECT_TRUE(action.done());
}
