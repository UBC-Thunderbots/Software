#include <gtest/gtest.h>

#include "software/ai/intent/intent.h"
#include "software/ai/intent/move_intent.h"

/**
 * This file contains the unit tests for the NavigatingIntent class (NOTE:
 * `NavigatingIntent` is virtual, so we use `MoveIntent` instead, but we're only testing
 * functionality of the `Intent` class)
 */

TEST(NavigatingIntentTest, test_set_and_get_motion_constraints)
{
    MoveIntent move_intent =
        MoveIntent(0, Point(), Angle(), 0.0, 0, DribblerEnable::OFF, MoveType::NORMAL,
                   AutochickType::NONE, BallCollisionType::AVOID);

    std::set<MotionConstraint> motion_constraint = {
        MotionConstraint::FRIENDLY_DEFENSE_AREA, MotionConstraint::CENTER_CIRCLE};
    move_intent.setMotionConstraints(motion_constraint);

    ASSERT_TRUE(move_intent.getNavigatorParams());
    EXPECT_EQ(motion_constraint, move_intent.getNavigatorParams()->motion_constraints);
}

TEST(NavigatingIntentTest, test_get_primitive_msg)
{
    MoveIntent move_intent =
        MoveIntent(0, Point(), Angle(), 0.0, 0, DribblerEnable::OFF, MoveType::NORMAL,
                   AutochickType::NONE, BallCollisionType::AVOID);

    EXPECT_TRUE(move_intent.getPrimitiveMsg().has_move());
}

TEST(NavigatingIntentTest, test_motion_constraints_inequality)
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

TEST(NavigatingIntentTest, test_equality)
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
