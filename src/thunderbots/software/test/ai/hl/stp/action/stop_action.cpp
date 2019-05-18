#include "ai/hl/stp/action/stop_action.h"

#include <gtest/gtest.h>

#include "ai/intent/stop_intent.h"


// StopAction should be yeilding stop_intents without coast
TEST(StopAction, stop_action_with_coast)
{
    Robot robot       = Robot(0, Point(10, 10), Vector(), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    StopAction action = StopAction();

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

// StopAction should be yeilding stop_intents with coast
TEST(StopAction, stop_action_coast)
{
    Robot robot       = Robot(0, Point(10, 10), Vector(), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    StopAction action = StopAction();

    auto intent_ptr = action.updateStateAndGetNextIntent(robot, true);

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
