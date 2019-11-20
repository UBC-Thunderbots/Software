#include <gtest/gtest.h>

#include "software/ai/hl/stp/tactic/test_tactics/move_test_tactic.h"

/**
 * This file contains the unit tests for the Tactic class (NOTE: `Tactic` is virtual, so
 * we use `MoveTestTactic` instead, but we're only testing the generic functionality of
 * the `Tactic` class)
 */

TEST(TacticTest, test_get_assigned_robot_with_no_robot_and_no_params)
{
    MoveTestTactic tactic = MoveTestTactic();
    auto robot            = tactic.getAssignedRobot();

    EXPECT_EQ(robot, std::nullopt);
}

TEST(TacticTest, test_get_assigned_robot_with_no_robot)
{
    MoveTestTactic tactic = MoveTestTactic();
    tactic.updateControlParams(Point());
    auto robot = tactic.getAssignedRobot();

    EXPECT_EQ(robot, std::nullopt);
}

TEST(TacticTest, test_get_assigned_robot_with_a_robot_assigned)
{
    MoveTestTactic tactic = MoveTestTactic();
    auto robot            = Robot(1, Point(1, 0), Vector(), Angle::threeQuarter(),
                       AngularVelocity::zero(), Timestamp::fromSeconds(0));
    tactic.updateRobot(robot);

    EXPECT_TRUE(tactic.getAssignedRobot());
    EXPECT_EQ(robot, *(tactic.getAssignedRobot()));
}

TEST(TacticTest, test_nullptr_returned_when_no_robot_assigned)
{
    MoveTestTactic tactic = MoveTestTactic();
    tactic.updateControlParams(Point());

    EXPECT_FALSE(tactic.getNextAction());
}

TEST(TacticTest, test_tactic_not_done_after_run_with_no_robot_assigned)
{
    MoveTestTactic tactic = MoveTestTactic();
    tactic.updateControlParams(Point());

    // Run the tactic several times
    for (int i = 0; i < 5; i++)
    {
        tactic.getNextAction();
    }

    EXPECT_FALSE(tactic.done());
}

TEST(TacticTest, test_tactic_does_not_prematurely_report_done)
{
    // This test is making sure we don't have any bugs with managing our coroutines that
    // cause the tactic to end before we would expect

    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));

    // Create a MoveTestTactic with the final destination very far from where the robot
    // currently is, so that we know the tactic should not report done
    MoveTestTactic tactic = MoveTestTactic();
    tactic.updateRobot(robot);
    tactic.updateControlParams(Point(3, 4));

    // Run the Tactic several times, leaving the robot and parameters as is so the
    // tactic should not approach a "done" state
    auto action_ptr = std::shared_ptr<Action>{};
    for (int i = 0; i < 10; i++)
    {
        action_ptr = tactic.getNextAction();
    }

    // Check an action was returned (the pointer is not null)
    EXPECT_TRUE(action_ptr);
    EXPECT_FALSE(tactic.done());
}

TEST(TacticTest, test_tactic_reports_done_at_same_time_nullptr_returned)
{
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));

    // Create a MoveTestTactic that wants to move the Robot to where it already is.
    // Therefore we expect the tactic to be done
    MoveTestTactic tactic = MoveTestTactic();
    tactic.updateRobot(robot);
    tactic.updateControlParams(Point());

    // The tactic should always return an Action the first time it is run to make sure we
    // are doing the right thing (and just don't happen to be in the "done state" at the
    // time while actually doing something else)
    auto action_ptr = tactic.getNextAction();
    EXPECT_TRUE(action_ptr);
    EXPECT_FALSE(tactic.done());

    // Subsequent calls should return a nullptr since the Tactic is done, and the
    // tactic should also report done()
    action_ptr = tactic.getNextAction();
    EXPECT_FALSE(action_ptr);
    EXPECT_TRUE(tactic.done());
}

TEST(TacticTest, test_tactic_restarts_when_set_to_loop_infinitely)
{
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));

    // Create a MoveTestTactic that wants to move the Robot to where it already is.
    // Therefore we expect the tactic to be done
    MoveTestTactic tactic = MoveTestTactic(true);
    tactic.updateRobot(robot);
    tactic.updateControlParams(Point());

    // Even though the Tactic should be done, we expect it to continue returning valid
    // Actions because it will be constantly restarting
    std::shared_ptr<Action> action_ptr;
    for (int i = 0; i < 5; i++)
    {
        action_ptr = tactic.getNextAction();
        EXPECT_TRUE(action_ptr);
        EXPECT_FALSE(tactic.done());
    }
}
