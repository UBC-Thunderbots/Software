/**
 * This file contains unit tests for the MoveSpin Intent class
 */

#include "ai/intent/movespin_intent.h"

#include <gtest/gtest.h>
#include <string.h>

TEST(MoveSpinIntentTest, intent_name_test)
{
    MoveSpinIntent movespin_intent = MoveSpinIntent(0, Point(), Angle::zero(), 0);

    EXPECT_EQ("MoveSpin Intent", movespin_intent.getIntentName());
}

// For equality operators, we only check for cases not covered in the Primitive tests,
// since Intents inherit from Primitives
TEST(MoveSpinIntentTest, test_equality_operator_intents_equal)
{
    MoveSpinIntent movespin_intent       = MoveSpinIntent(0, Point(), Angle::zero(), 0);
    MoveSpinIntent movespin_intent_other = MoveSpinIntent(0, Point(), Angle::zero(), 0);

    EXPECT_EQ(movespin_intent, movespin_intent_other);
}

TEST(MoveSpinIntentTest, test_inequality_operator_with_mismatched_priorities)
{
    MoveSpinIntent movespin_intent       = MoveSpinIntent(0, Point(), Angle::zero(), 1);
    MoveSpinIntent movespin_intent_other = MoveSpinIntent(0, Point(), Angle::zero(), 3);

    EXPECT_NE(movespin_intent, movespin_intent_other);
}
