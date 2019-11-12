/**
 * This file contains the unit tests for the Intent class (NOTE: `Intent` is virtual, so
 * we use `MoveIntent` instead, but we're only testing functionality of the `Intent`
 * class)
 */

#include "software/ai/intent/intent.h"

#include <gtest/gtest.h>

#include "software/ai/intent/move_intent.h"


TEST(IntentTest, test_get_priority)
{
    MoveIntent move_intent =
        MoveIntent(0, Point(), Angle(), 0.0, 2, DribblerEnable::OFF, MoveType::NORMAL,
                   AutokickType::NONE, BallCollisionType::AVOID);

    EXPECT_EQ(2, move_intent.getPriority());
}

TEST(IntentTest, test_set_priority)
{
    MoveIntent move_intent =
        MoveIntent(0, Point(), Angle(), 0.0, 0, DribblerEnable::OFF, MoveType::NORMAL,
                   AutokickType::NONE, BallCollisionType::AVOID);
    move_intent.setPriority(7);

    EXPECT_EQ(7, move_intent.getPriority());
}

TEST(IntentTest, test_set_and_get_motion_constraints)
{
    MoveIntent move_intent =
        MoveIntent(0, Point(), Angle(), 0.0, 0, DribblerEnable::OFF, MoveType::NORMAL,
                   AutokickType::NONE, BallCollisionType::AVOID);

    std::set<MotionConstraint> motion_constraint = {
        MotionConstraint::FRIENDLY_DEFENSE_AREA, MotionConstraint::CENTER_CIRCLE};
    move_intent.setMotionConstraints(motion_constraint);

    EXPECT_EQ(motion_constraint, move_intent.getMotionConstraints());
}
