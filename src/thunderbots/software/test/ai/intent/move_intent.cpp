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
