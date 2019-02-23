#include "ai/hl/stp/tactic/move_tactic.h"

#include <gtest/gtest.h>

#include <boost/coroutine2/all.hpp>

#include "ai/intent/move_intent.h"
#include "test/test_util/test_util.h"

TEST(MoveTacticTest, robot_far_from_destination)
{
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    MoveTactic tactic = MoveTactic(robot);

    auto intent_ptr =
        tactic.updateStateAndGetNextIntent(robot, Point(1, 0), Angle::quarter(), 1.0);

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
    MoveTactic tactic = MoveTactic(robot);

    // We call the Tactic twice. The first time the Intent will always be returned to
    // ensure the Robot is doing the right thing. In all future calls, the action will be
    // done and so will return a null pointer
    auto intent_ptr =
        tactic.updateStateAndGetNextIntent(robot, Point(0, 0), Angle::zero(), 0.0);
    intent_ptr =
        tactic.updateStateAndGetNextIntent(robot, Point(0, 0), Angle::zero(), 0.0);

    // We expect the resultant pointer to be null since the Action is done and has no more
    // Intents to return, since the robot is already at the destination
    EXPECT_FALSE(intent_ptr);
}

TEST(MoveActionTest, test_action_does_not_prematurely_report_done)
{
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    MoveTactic tactic = MoveTactic(robot);

    // Run the Tactic several times
    auto intent_ptr = std::unique_ptr<Intent>{};
    for (int i = 0; i < 10; i++)
    {
        intent_ptr =
            tactic.updateStateAndGetNextIntent(robot, Point(1, 0), Angle::quarter(), 1.0);
    }

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
}

TEST(MoveActionTest, test_action_done)
{
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    MoveTactic tactic = MoveTactic(robot);

    // Run the Tactic several times
    auto intent_ptr = std::unique_ptr<Intent>{};
    for (int i = 0; i < 5; i++)
    {
        intent_ptr =
            tactic.updateStateAndGetNextIntent(robot, Point(0, 0), Angle::zero(), 0.0);
    }

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(tactic.done());
}

TEST(MoveActionTest, test_action_not_done)
{
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    MoveTactic tactic = MoveTactic(robot);

    // Run the Action several times
    auto intent_ptr = std::unique_ptr<Intent>{};
    for (int i = 0; i < 5; i++)
    {
        intent_ptr =
            tactic.updateStateAndGetNextIntent(robot, Point(2, -1), Angle::zero(), 0.0);
    }

    // Check an intent was returned (the pointer is not null)
    EXPECT_FALSE(tactic.done());
}

TEST(MoveActionTest, test_evaluate_robot_function)
{
    Field field = ::Test::TestUtil::createSSLDivBField();
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    MoveTactic tactic = MoveTactic(robot);

    auto intent_ptr =
        tactic.updateStateAndGetNextIntent(robot, Point(3, -4), Angle::zero(), 0.0);

    EXPECT_EQ(5 / field.totalLength(), tactic.calculateRobotCost(robot, field));
}
