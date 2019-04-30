#include "ai/hl/stp/action/dribble_action.h"

#include <gtest/gtest.h>

#include "ai/intent/dribble_intent.h"

TEST(DribbleActionTest, robot_far_from_destination)
{
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    DribbleAction action = DribbleAction(0.05);

    auto intent_ptr = action.updateStateAndGetNextIntent(robot, Point(1, 0),
                                                         Angle::quarter(), 10000, false);

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    DribbleIntent dribble_intent = dynamic_cast<DribbleIntent &>(*intent_ptr);
    EXPECT_EQ(0, dribble_intent.getRobotId());
    EXPECT_EQ(Point(1, 0), dribble_intent.getDestination());
    EXPECT_EQ(Angle::quarter(), dribble_intent.getFinalAngle());
    EXPECT_EQ(10000, dribble_intent.getRpm());
    EXPECT_FALSE(dribble_intent.isSmallKickAllowed());
}

TEST(DribbleActionTest, robot_at_destination)
{
    Robot robot = Robot(0, Point(), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    DribbleAction action = DribbleAction(0.02);

    // We call the action twice. The first time the Intent will always be returned to
    // ensure the Robot is doing the right thing. In all future calls, the action will be
    // done and so will return a null pointer
    auto intent_ptr = action.updateStateAndGetNextIntent(robot, Point(0, 0),
                                                         Angle::zero(), 10000, false);
    intent_ptr = action.updateStateAndGetNextIntent(robot, Point(0, 0), Angle::zero(),
                                                    10000, false);

    EXPECT_TRUE(action.done());
}

TEST(DribbleActionTest, test_action_does_not_prematurely_report_done)
{
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    DribbleAction action = DribbleAction(0.05);

    // Run the Action several times
    auto intent_ptr = std::unique_ptr<Intent>{};
    for (int i = 0; i < 10; i++)
    {
        intent_ptr = action.updateStateAndGetNextIntent(robot, Point(1, 0),
                                                        Angle::quarter(), 6000, true);
    }

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());
}

TEST(DribbleActionTest, robot_far_from_destination_small_kick_allowed)
{
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    DribbleAction action = DribbleAction(0.05);

    auto intent_ptr = action.updateStateAndGetNextIntent(robot, Point(1, 0),
                                                         Angle::quarter(), 20000, true);

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    DribbleIntent dribble_intent = dynamic_cast<DribbleIntent &>(*intent_ptr);
    EXPECT_EQ(0, dribble_intent.getRobotId());
    EXPECT_EQ(Point(1, 0), dribble_intent.getDestination());
    EXPECT_EQ(Angle::quarter(), dribble_intent.getFinalAngle());
    EXPECT_EQ(20000, dribble_intent.getRpm());
    EXPECT_TRUE(dribble_intent.isSmallKickAllowed());
}
