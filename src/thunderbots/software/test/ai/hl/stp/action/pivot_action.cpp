#include "ai/hl/stp/action/pivot_action.h"

#include <gtest/gtest.h>

#include "ai/intent/move_intent.h"
#include "ai/intent/pivot_intent.h"


// PivotAction should be yielding move_intents as the robot is too far away from orbit
TEST(PivotActionTest, robot_too_far_from_orbit)
{
    Robot robot        = Robot(0, Point(10, 10), Vector(), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PivotAction action = PivotAction();

    auto intent_ptr = action.updateStateAndGetNextIntent(
        robot, Point(0, 0), Angle::zero(), Angle::ofRadians(2.0), true);

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    try
    {
        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
        EXPECT_EQ(0, move_intent.getRobotId());
    }
    catch (...)
    {
        ADD_FAILURE()
            << "Move intent not returned by pivot action, even though robot was far away!";
    }
}

// PivotAction should be yielding pivot intents as the robot is close enough to the ball
TEST(PivotActionTest, yield_pivotintent_when_close_to_ball)
{
    Robot robot        = Robot(0, Point(0.5, 0), Vector(), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PivotAction action = PivotAction();

    auto intent_ptr = action.updateStateAndGetNextIntent(
        robot, Point(0, 0), Angle::half(), Angle::ofRadians(2.0), true);

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    try
    {
        PivotIntent pivot_intent = dynamic_cast<PivotIntent &>(*intent_ptr);
        EXPECT_EQ(0, pivot_intent.getRobotId());
    }
    catch (...)
    {
        ADD_FAILURE()
            << "Pivot intent not returned by pivot action, even though robot was in orbit!";
    }
}

// PivotAction should be yielding move intents when far from the ball
TEST(PivotActionTest, yield_moveintent_when_far_from_ball)
{
    Robot robot        = Robot(0, Point(1.5, 0), Vector(), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PivotAction action = PivotAction();

    auto intent_ptr = action.updateStateAndGetNextIntent(
        robot, Point(0, 0), Angle::half(), Angle::ofRadians(2.0), true);

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    try
    {
        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
        EXPECT_EQ(0, move_intent.getRobotId());
    }
    catch (...)
    {
        ADD_FAILURE()
            << "Move intent not returned by pivot action, even though robot was far away from the ball!";
    }
}
