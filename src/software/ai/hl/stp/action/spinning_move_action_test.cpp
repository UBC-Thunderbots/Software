#include "software/ai/hl/stp/action/spinning_move_action.h"

#include <google/protobuf/util/message_differencer.h>
#include <gtest/gtest.h>

#include "software/ai/intent/spinning_move_intent.h"
#include "software/proto/message_translation/tbots_protobuf.h"

TEST(SpinningMoveActionTest, getDestination)
{
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    SpinningMoveAction action = SpinningMoveAction(false, 0.05);

    action.updateControlParams(robot, Point(7, 13), AngularVelocity::quarter(), 1.0);

    EXPECT_EQ(Point(7, 13), action.getDestination());
}

TEST(SpinningMoveActionTest, robot_far_from_destination)
{
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    SpinningMoveAction action = SpinningMoveAction(false, 0.05);

    action.updateControlParams(robot, Point(1, 0), AngularVelocity::quarter(), 1.0);
    auto intent_ptr = action.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    SpinningMoveIntent spinning_move_intent =
        dynamic_cast<SpinningMoveIntent &>(*intent_ptr);
    EXPECT_EQ(0, spinning_move_intent.getRobotId());

    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        spinning_move_intent.getPrimitive().spinning_move().destination(),
        *createPointProto(Point(1, 0))));
    EXPECT_EQ(1.0,
              spinning_move_intent.getPrimitive().spinning_move().final_speed_m_per_s());
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        spinning_move_intent.getPrimitive().spinning_move().angular_velocity(),
        *createAngularVelocityProto(AngularVelocity::quarter())));
}

TEST(SpinningMoveActionTest, robot_at_destination)
{
    Robot robot = Robot(0, Point(), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    SpinningMoveAction action = SpinningMoveAction(false, 0.02);

    // We call the action twice. The first time the Intent will always be returned to
    // ensure the Robot is doing the right thing. In all future calls, the action will be
    // done and so will return a null pointer
    action.updateControlParams(robot, Point(0, 0), AngularVelocity::full(), 0);
    action.getNextIntent();
    action.getNextIntent();

    EXPECT_TRUE(action.done());
}

TEST(SpinningMoveActionTest, test_action_does_not_prematurely_report_done)
{
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    SpinningMoveAction action = SpinningMoveAction(false, 0.05);

    // Run the Action several times
    for (int i = 0; i < 10; i++)
    {
        action.updateControlParams(robot, Point(1, 0), AngularVelocity::quarter(), 1.0);
    }
    auto intent_ptr = action.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());
}
