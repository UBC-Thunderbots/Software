#include "software/ai/intent/intent.h"

#include <gtest/gtest.h>

#include "software/ai/intent/move_intent.h"
#include "software/ai/intent/stop_intent.h"

/**
 * This file contains the unit tests for the Intent class (NOTE: `Intent` is virtual, so
 * we use `MoveIntent` and `StopIntent` instead, but we're only testing functionality of
 * the `Intent` class)
 */

TEST(IntentTest, test_get_priority)
{
    MoveIntent move_intent =
        MoveIntent(0, Point(), Angle(), 0.0, 2, DribblerEnable::OFF, MoveType::NORMAL,
                   AutochickType::NONE, BallCollisionType::AVOID);

    EXPECT_EQ(2, move_intent.getPriority());
}

TEST(IntentTest, test_set_priority)
{
    MoveIntent move_intent =
        MoveIntent(0, Point(), Angle(), 0.0, 0, DribblerEnable::OFF, MoveType::NORMAL,
                   AutochickType::NONE, BallCollisionType::AVOID);
    move_intent.setPriority(7);

    EXPECT_EQ(7, move_intent.getPriority());
}

TEST(IntentTest, test_set_and_get_motion_constraints)
{
    MoveIntent move_intent =
        MoveIntent(0, Point(), Angle(), 0.0, 0, DribblerEnable::OFF, MoveType::NORMAL,
                   AutochickType::NONE, BallCollisionType::AVOID);

    std::set<MotionConstraint> motion_constraint = {
        MotionConstraint::FRIENDLY_DEFENSE_AREA, MotionConstraint::CENTER_CIRCLE};
    move_intent.setMotionConstraints(motion_constraint);

    EXPECT_EQ(motion_constraint, move_intent.getMotionConstraints());
}

TEST(IntentTest, test_get_primitive_msg)
{
    MoveIntent move_intent =
        MoveIntent(0, Point(), Angle(), 0.0, 0, DribblerEnable::OFF, MoveType::NORMAL,
                   AutochickType::NONE, BallCollisionType::AVOID);

    EXPECT_TRUE(move_intent.getPrimitiveMsg().has_move());
    EXPECT_TRUE(move_intent.getUpdatedPrimitiveMsg(Point(), 0).has_move());
}

TEST(IntentTest, test_get_robot_id)
{
    MoveIntent move_intent =
        MoveIntent(1, Point(), Angle(), 0.0, 0, DribblerEnable::OFF, MoveType::NORMAL,
                   AutochickType::NONE, BallCollisionType::AVOID);

    EXPECT_EQ(move_intent.getRobotId(), 1);
}

TEST(IntentTest, test_get_navigator_params_stop)
{
    StopIntent stop_intent = StopIntent(0, false, 1);
    EXPECT_FALSE(stop_intent.getNavigatorParams());
}

TEST(IntentTest, test_robot_id_inequality)
{
    StopIntent stop_intent_1 = StopIntent(0, false, 1);
    StopIntent stop_intent_2 = StopIntent(1, false, 1);

    EXPECT_NE(stop_intent_1, stop_intent_2);
}

TEST(IntentTest, test_priority_inequality)
{
    StopIntent stop_intent_1 = StopIntent(1, false, 2);
    StopIntent stop_intent_2 = StopIntent(1, false, 1);

    EXPECT_NE(stop_intent_1, stop_intent_2);
}

TEST(IntentTest, test_motion_constraints_inequality)
{
    MoveIntent move_intent_1 =
        MoveIntent(0, Point(), Angle(), 0.0, 0, DribblerEnable::OFF, MoveType::NORMAL,
                   AutochickType::NONE, BallCollisionType::AVOID);

    std::set<MotionConstraint> motion_constraint_1 = {
        MotionConstraint::FRIENDLY_DEFENSE_AREA, MotionConstraint::CENTER_CIRCLE};
    move_intent_1.setMotionConstraints(motion_constraint_1);

    MoveIntent move_intent_2 =
        MoveIntent(0, Point(), Angle(), 0.0, 0, DribblerEnable::OFF, MoveType::NORMAL,
                   AutochickType::NONE, BallCollisionType::AVOID);

    std::set<MotionConstraint> motion_constraint_2 = {MotionConstraint::CENTER_CIRCLE};
    move_intent_2.setMotionConstraints(motion_constraint_2);

    EXPECT_NE(move_intent_1, move_intent_2);
}

TEST(IntentTest, test_equality)
{
    MoveIntent move_intent_1 =
        MoveIntent(0, Point(), Angle(), 0.0, 0, DribblerEnable::OFF, MoveType::NORMAL,
                   AutochickType::NONE, BallCollisionType::AVOID);

    std::set<MotionConstraint> motion_constraint = {
        MotionConstraint::FRIENDLY_DEFENSE_AREA, MotionConstraint::CENTER_CIRCLE};
    move_intent_1.setMotionConstraints(motion_constraint);

    MoveIntent move_intent_2 =
        MoveIntent(0, Point(), Angle(), 0.0, 0, DribblerEnable::OFF, MoveType::NORMAL,
                   AutochickType::NONE, BallCollisionType::AVOID);

    move_intent_2.setMotionConstraints(motion_constraint);

    EXPECT_EQ(move_intent_1, move_intent_2);
}
