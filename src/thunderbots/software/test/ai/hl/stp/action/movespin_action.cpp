#include "ai/hl/stp/action/movespin_action.h"

#include <gtest/gtest.h>

#include "ai/intent/movespin_intent.h"

TEST(MoveSpinActionTest, robot_far_from_destination)
{
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    MoveSpinAction action = MoveSpinAction(0.05, false);

    auto intent_ptr = action.updateStateAndGetNextIntent(robot, Point(1, 0),
                                                         AngularVelocity::quarter(), 1.0);

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    MoveSpinIntent movespin_intent = dynamic_cast<MoveSpinIntent &>(*intent_ptr);
    EXPECT_EQ(0, movespin_intent.getRobotId());
    EXPECT_EQ(Point(1, 0), movespin_intent.getDestination());
    EXPECT_EQ(AngularVelocity::quarter(), movespin_intent.getAngularVelocity());
    EXPECT_DOUBLE_EQ(1.0, movespin_intent.getFinalSpeed());
}

TEST(MoveSpinActionTest, robot_at_destination)
{
    Robot robot = Robot(0, Point(), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    MoveSpinAction action = MoveSpinAction(0.02, false);

    // We call the action twice. The first time the Intent will always be returned to
    // ensure the Robot is doing the right thing. In all future calls, the action will be
    // done and so will return a null pointer
    auto intent_ptr = action.updateStateAndGetNextIntent(robot, Point(0, 0),
                                                         AngularVelocity::full(), 0);
    intent_ptr      = action.updateStateAndGetNextIntent(robot, Point(0, 0),
                                                    AngularVelocity::full(), 0);

    EXPECT_TRUE(action.done());
}

TEST(MoveSpinActionTest, test_action_does_not_prematurely_report_done)
{
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    MoveSpinAction action = MoveSpinAction(0.05, false);

    // Run the Action several times
    auto intent_ptr = std::unique_ptr<Intent>{};
    for (int i = 0; i < 10; i++)
    {
        intent_ptr = action.updateStateAndGetNextIntent(robot, Point(1, 0),
                                                        AngularVelocity::quarter(), 1.0);
    }

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());
}
