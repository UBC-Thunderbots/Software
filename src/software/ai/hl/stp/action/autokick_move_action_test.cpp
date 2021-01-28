#include "software/ai/hl/stp/action/autokick_move_action.h"

#include <gtest/gtest.h>

#include "software/ai/intent/autokick_move_intent.h"

TEST(AutokickMoveActionTest, test_getters)
{
    Robot robot = Robot(13, Point(1, 2), Vector(3, 4), Angle::fromDegrees(5),
                        AngularVelocity::fromDegrees(6), Timestamp::fromSeconds(7));
    AutokickMoveAction action = AutokickMoveAction(false, 0.05, Angle());

    action.updateControlParams(robot, Point(11, 12), Angle::quarter(), 9.1,
                               DribblerMode::MAX_FORCE, 1.2, BallCollisionType::AVOID);

    EXPECT_EQ(Point(11, 12), action.getDestination());
    EXPECT_EQ(1.2, action.getKickSpeed());
    EXPECT_EQ(Angle::quarter(), action.getFinalOrientation());
    EXPECT_EQ(9.1, action.getFinalSpeed());
    EXPECT_EQ(DribblerMode::MAX_FORCE, action.getDribblerMode());
}

TEST(AutokickMoveActionTest, robot_far_from_destination)
{
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    AutokickMoveAction action = AutokickMoveAction(false, 0.05, Angle());

    action.updateControlParams(robot, Point(1, 0), Angle::quarter(), 1.0,
                               DribblerMode::OFF, 2.4, BallCollisionType::AVOID);
    auto intent_ptr = action.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    AutokickMoveIntent autokick_move_intent =
        dynamic_cast<AutokickMoveIntent &>(*intent_ptr);
    EXPECT_EQ(0, autokick_move_intent.getRobotID());
    EXPECT_EQ(Point(1, 0), autokick_move_intent.getDestination());
    EXPECT_EQ(Angle::quarter(), autokick_move_intent.getFinalAngle());
    EXPECT_EQ(1.0, autokick_move_intent.getFinalSpeed());
    EXPECT_EQ(2.4, autokick_move_intent.getKickSpeed());
    EXPECT_TRUE(autokick_move_intent.getDribblerMode() == DribblerMode::OFF);
}

TEST(AutokickMoveActionTest, robot_at_destination)
{
    Robot robot = Robot(0, Point(), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    AutokickMoveAction action = AutokickMoveAction(false, 0.02, Angle());

    // We call the action twice. The first time the Intent will always be returned to
    // ensure the Robot is doing the right thing. In all future calls, the action will be
    // done and so will return a null pointer
    action.updateControlParams(robot, Point(0, 0), Angle::zero(), 0.0, DribblerMode::OFF,
                               2.4, BallCollisionType::AVOID);
    action.getNextIntent();
    action.getNextIntent();

    EXPECT_TRUE(action.done());
}

TEST(AutokickMoveActionTest, test_action_does_not_prematurely_report_done)
{
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    AutokickMoveAction action = AutokickMoveAction(false, 0.05, Angle());

    // Run the Action several times
    action.updateControlParams(robot, Point(1, 0), Angle::quarter(), 1.0,
                               DribblerMode::OFF, 1.3, BallCollisionType::AVOID);
    for (int i = 0; i < 10; i++)
    {
        action.getNextIntent();
    }
    auto intent_ptr = action.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());
}

TEST(AutokickMoveActionTest, test_action_does_not_prematurely_report_done_angle_threshold)
{
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    AutokickMoveAction action = AutokickMoveAction(false, 0.05, Angle::fromDegrees(0.01));

    // Run the Action several times
    action.updateControlParams(robot, Point(0, 0), Angle::quarter(), 1.0,
                               DribblerMode::OFF, 1.4, BallCollisionType::AVOID);
    for (int i = 0; i < 10; i++)
    {
        action.getNextIntent();
    }
    auto intent_ptr = action.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());
}
TEST(AutokickMoveActionTest, test_action_finishes_within_orientation_threshold)
{
    Robot robot = Robot(0, Point(0, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    AutokickMoveAction action = AutokickMoveAction(false, 0.05, Angle::fromDegrees(359));

    // Run the Action several times
    action.updateControlParams(robot, Point(0, 0), Angle::quarter(), 1.0,
                               DribblerMode::OFF, 1.1, BallCollisionType::AVOID);
    for (int i = 0; i < 10; i++)
    {
        action.getNextIntent();
    }
    auto intent_ptr = action.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_FALSE(intent_ptr);
    EXPECT_TRUE(action.done());
}

TEST(AutokickMoveActionTest, robot_far_from_destination_autokick_turned_on)
{
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    AutokickMoveAction action = AutokickMoveAction(false, 0.05, Angle());

    action.updateControlParams(robot, Point(1, 0), Angle::quarter(), 1.0,
                               DribblerMode::OFF, 1.5, BallCollisionType::AVOID);
    auto intent_ptr = action.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    AutokickMoveIntent move_intent = dynamic_cast<AutokickMoveIntent &>(*intent_ptr);
    EXPECT_EQ(0, move_intent.getRobotID());
    EXPECT_EQ(Point(1, 0), move_intent.getDestination());
    EXPECT_EQ(Angle::quarter(), move_intent.getFinalAngle());
    EXPECT_EQ(1.0, move_intent.getFinalSpeed());
    EXPECT_EQ(1.5, move_intent.getKickSpeed());
    EXPECT_EQ(move_intent.getDribblerMode(), DribblerMode::OFF);
}

TEST(AutokickMoveActionTest, robot_far_from_destination_dribble_turned_on)
{
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    AutokickMoveAction action = AutokickMoveAction(false, 0.05, Angle());

    action.updateControlParams(robot, Point(1, 0), Angle::quarter(), 1.0,
                               DribblerMode::MAX_FORCE, 4.3, BallCollisionType::AVOID);
    auto intent_ptr = action.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    AutokickMoveIntent move_intent = dynamic_cast<AutokickMoveIntent &>(*intent_ptr);
    EXPECT_EQ(0, move_intent.getRobotID());
    EXPECT_EQ(Point(1, 0), move_intent.getDestination());
    EXPECT_EQ(Angle::quarter(), move_intent.getFinalAngle());
    EXPECT_EQ(1.0, move_intent.getFinalSpeed());
    EXPECT_EQ(4.3, move_intent.getKickSpeed());
    EXPECT_TRUE(move_intent.getDribblerMode() == DribblerMode::MAX_FORCE);
}
