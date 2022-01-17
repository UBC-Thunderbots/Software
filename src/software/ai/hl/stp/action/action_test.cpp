#include <gtest/gtest.h>

#include "software/ai/hl/stp/action/test_actions/move_test_action.h"
#include "software/ai/intent/move_intent.h"

/**
 * This file contains the unit tests for the Action class (NOTE: `Action` is virtual, so
 * we use `MoveTestAction` instead, but we're only testing the generic functionality of
 * the `Tactic` class)
 */

TEST(ActionTest, test_action_reports_done_at_same_time_nullptr_returned)
{
    Robot robot = Robot(0, Point(0, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    MoveTestAction action = MoveTestAction(0.05, false);

    // The first time the Action runs it will always return an Intent to make sure we
    // are doing the correct thing
    action.updateControlParams(robot, Point());
    auto intent_ptr = action.getNextIntent();
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    // For subsequent calls, we expect the Action to be done (in this case)
    // We make sure that when a nullptr is returned, the action also evaluates to "done"
    // This is important since higher-level functionality relies on the Action::done()
    // function but returning nullptr values out of sync with this done() function could
    // cause problems
    intent_ptr = action.getNextIntent();
    EXPECT_FALSE(intent_ptr);
    EXPECT_TRUE(action.done());
}

TEST(ActionTest, getRobot)
{
    Robot robot           = Robot(13, Point(1, 2), Vector(3, 4), Angle::fromDegrees(5),
                        AngularVelocity::fromDegrees(6), Timestamp::fromSeconds(7));
    MoveTestAction action = MoveTestAction(0.05, false);
    action.updateControlParams(robot, Point());

    std::optional<Robot> robot_opt = action.getRobot();
    EXPECT_TRUE(robot_opt.has_value());
    EXPECT_EQ(robot, robot_opt.value());
}

TEST(ActionTest, restart_after_done_makes_done_false)
{
    Robot robot = Robot(0, Point(0, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    MoveTestAction action = MoveTestAction(0.05, false);

    // The first time the Action runs it will always return an Intent to make sure we
    // are doing the correct thing
    action.updateControlParams(robot, Point(0, 0));
    auto intent_ptr = action.getNextIntent();
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    // For subsequent calls, we expect the Action to be done (in this case)
    intent_ptr = action.getNextIntent();
    EXPECT_FALSE(intent_ptr);
    EXPECT_TRUE(action.done());

    // if we restart the action, we expect it to return an intent and
    // to evaluate "done" as false
    action.restart();
    intent_ptr = action.getNextIntent();
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());
}

TEST(ActionTest, action_with_loop_forever_does_not_return_nullptr)
{
    Robot robot = Robot(0, Point(0, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    MoveTestAction action = MoveTestAction(0.05, true);
    action.updateControlParams(robot, Point(0, 0));

    // run action multiple times, should return an intent every time
    for (int i = 0; i < 5; i++)
    {
        auto intent_ptr = action.getNextIntent();
        EXPECT_TRUE(intent_ptr);
        EXPECT_FALSE(action.done());
    }

    // change point destination and expect intent to be returned every time
    action.updateControlParams(robot, Point(1, 1));
    for (int i = 0; i < 5; i++)
    {
        auto intent_ptr = action.getNextIntent();
        EXPECT_TRUE(intent_ptr);
        EXPECT_FALSE(action.done());
    }
}
