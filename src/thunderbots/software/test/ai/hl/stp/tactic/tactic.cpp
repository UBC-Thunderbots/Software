#include <gtest/gtest.h>

#include "ai/hl/stp/tactic/move_tactic.h"

/**
 * This file contains the unit tests for the Tactic class (NOTE: `Tactic` is virtual, so
 * we use `MoveTactic` instead, but we're only testing the generic functionality of
 * the `Tactic` class)
 */

TEST(TacticTest, test_get_assigned_robot_with_no_robot_and_no_params)
{
    MoveTactic tactic = MoveTactic();
    auto robot        = tactic.getAssignedRobot();

    EXPECT_EQ(robot, std::nullopt);
}

TEST(TacticTest, test_get_assigned_robot_with_no_robot)
{
    MoveTactic tactic = MoveTactic();
    tactic.updateParams(Point(), Angle::quarter(), 1.2);
    auto robot = tactic.getAssignedRobot();

    EXPECT_EQ(robot, std::nullopt);
}

TEST(TacticTest, test_get_assigned_robot_with_a_robot_assigned)
{
    MoveTactic tactic = MoveTactic();
    auto robot        = Robot(1, Point(1, 0), Vector(), Angle::threeQuarter(),
                       AngularVelocity::zero(), Timestamp::fromSeconds(0));
    tactic.updateRobot(robot);

    EXPECT_TRUE(tactic.getAssignedRobot());
    EXPECT_EQ(robot, *(tactic.getAssignedRobot()));
}

TEST(TacticTest, test_nullptr_returned_when_no_robot_assigned)
{
    MoveTactic tactic = MoveTactic();
    tactic.updateParams(Point(), Angle::quarter(), 1.2);

    EXPECT_FALSE(tactic.getNextIntent());
}

TEST(TacticTest, test_tactic_not_done_after_run_with_no_robot_assigned)
{
    MoveTactic tactic = MoveTactic();
    tactic.updateParams(Point(), Angle::quarter(), 1.2);

    // Run the tactic several times
    for (int i = 0; i < 5; i++)
    {
        tactic.getNextIntent();
    }

    EXPECT_FALSE(tactic.done());
}

TEST(TacticTest, test_tactic_does_not_prematurely_report_done)
{
    // This test is making sure we don't have any bugs with managing our coroutines that
    // cause the tactic to end before we would expect

    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));

    // Create a MoveTactic with the final destination very far from where the robot
    // currently is, so that we know the tactic should not report done
    MoveTactic tactic = MoveTactic();
    tactic.updateRobot(robot);
    tactic.updateParams(Point(3, 4), Angle::quarter(), 2.2);

    // Run the Tactic several times, leaving the robot and parameters as is so the
    // tactic should not approach a "done" state
    auto intent_ptr = std::unique_ptr<Intent>{};
    for (int i = 0; i < 10; i++)
    {
        intent_ptr = tactic.getNextIntent();
    }

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(tactic.done());
}

TEST(TacticTest, test_tactic_reports_done_at_same_time_nullptr_returned)
{
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));

    // Create a MoveTactic that wants to move the Robot to where it already is.
    // Therefore we expect the tactic to be done
    MoveTactic tactic = MoveTactic();
    tactic.updateRobot(robot);
    tactic.updateParams(Point(), Angle::zero(), 0.0);

    // The tactic should always return an Intent the first time it is run to make sure we
    // are doing the right thing (and just don't happen to be in the "done state" at the
    // time while actually doing something else)
    auto intent_ptr = tactic.getNextIntent();
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(tactic.done());

    // Subsequent calls should return a nullptr since the Tactic is done, and the
    // tactic should also report done()
    intent_ptr = tactic.getNextIntent();
    EXPECT_FALSE(intent_ptr);
    EXPECT_TRUE(tactic.done());
}

TEST(TacticTest, test_tactic_restarts_when_set_to_loop_infinitely)
{
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));

    // Create a MoveTactic that wants to move the Robot to where it already is.
    // Therefore we expect the tactic to be done
    MoveTactic tactic = MoveTactic(true);
    tactic.updateRobot(robot);
    tactic.updateParams(Point(), Angle::zero(), 0.0);

    // Even though the Tactic should be done, we expect it to continue returning valid
    // Intents because it will be constantly restarting
    std::unique_ptr<Intent> intent_ptr;
    for (int i = 0; i < 5; i++)
    {
        intent_ptr = tactic.getNextIntent();
        EXPECT_TRUE(intent_ptr);
        EXPECT_FALSE(tactic.done());
    }
}
