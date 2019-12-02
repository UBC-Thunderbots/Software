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
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    MoveTestAction action = MoveTestAction(0.05);

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
    MoveTestAction action = MoveTestAction(0.05);
    action.updateControlParams(robot, Point());

    std::optional<Robot> robot_opt = action.getRobot();
    EXPECT_TRUE(robot_opt.has_value());
    EXPECT_EQ(robot, *robot_opt);
}
