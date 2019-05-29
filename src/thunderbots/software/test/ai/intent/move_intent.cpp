/**
 * This file contains unit tests for the Move Intent class
 */

#include "ai/intent/move_intent.h"

#include <gtest/gtest.h>
#include <string.h>

TEST(MoveIntentTest, intent_name_test)
{
    MoveIntent move_intent = MoveIntent(0, Point(), Angle::zero(), 0, 0);

    EXPECT_EQ("Move Intent", move_intent.getIntentName());
}

// For equality operators, we only check for cases not covered in the Primitive tests,
// since Intents inherit from Primitives
TEST(MoveIntentTest, test_equality_operator_intents_equal)
{
    MoveIntent move_intent       = MoveIntent(0, Point(), Angle::zero(), 0, 0);
    MoveIntent move_intent_other = MoveIntent(0, Point(), Angle::zero(), 0, 0);

    EXPECT_EQ(move_intent, move_intent_other);
}

TEST(MoveIntentTest, test_inequality_operator_with_mismatched_priorities)
{
    MoveIntent move_intent       = MoveIntent(0, Point(), Angle::zero(), 0, 1);
    MoveIntent move_intent_other = MoveIntent(0, Point(), Angle::zero(), 0, 3);

    EXPECT_NE(move_intent, move_intent_other);
}

TEST(MoveIntentTest, test_default_moveflags)
{
    MoveIntent move_intent = MoveIntent(0, Point(), Angle::zero(), 0, 1);

    EXPECT_EQ(move_intent.getMoveFlags(), MoveFlags::NONE);
}

TEST(MoveIntentTest, test_get_set_moveflags)
{
    MoveIntent move_intent = MoveIntent(0, Point(), Angle::zero(), 0, 1);
    MoveFlags flags        = MoveFlags::AVOID_THEM_DEFENSE | MoveFlags::AVOID_BALL_STOP;

    move_intent.setMoveFlags(flags);

    EXPECT_EQ(move_intent.getMoveFlags(), flags);
}

TEST(MoveIntentTest, test_invalid_moveflag)
{
    MoveIntent move_intent = MoveIntent(0, Point(), Angle::zero(), 0, 1);

    MoveFlags invalid_flag = MoveFlags::STAY_OWN_HALF | static_cast<MoveFlags>(0x12345);

    EXPECT_FALSE(isMoveFlagValid(invalid_flag));
}
