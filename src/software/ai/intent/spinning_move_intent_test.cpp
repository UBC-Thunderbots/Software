#include "software/ai/intent/spinning_move_intent.h"

#include <gtest/gtest.h>
#include <string.h>

TEST(SpinningMoveIntentTest, intent_name_test)
{
    SpinningMoveIntent spinning_move_intent =
        SpinningMoveIntent(0, Point(), Angle::zero(), 1.0, 0);

    EXPECT_EQ("SpinningMove Intent", spinning_move_intent.getIntentName());
}

// For equality operators, we only check for cases not covered in the Primitive tests,
// since Intents inherit from Primitives
TEST(SpinningMoveIntentTest, test_equality_operator_intents_equal)
{
    SpinningMoveIntent spinning_move_intent =
        SpinningMoveIntent(0, Point(), Angle::zero(), 1.0, 0);
    SpinningMoveIntent spinning_move_intent_other =
        SpinningMoveIntent(0, Point(), Angle::zero(), 1.0, 0);

    EXPECT_EQ(spinning_move_intent, spinning_move_intent_other);
}

TEST(SpinningMoveIntentTest, test_inequality_operator_with_mismatched_priorities)
{
    SpinningMoveIntent spinning_move_intent =
        SpinningMoveIntent(0, Point(), Angle::zero(), 1.0, 1);
    SpinningMoveIntent spinning_move_intent_other =
        SpinningMoveIntent(0, Point(), Angle::zero(), 1.0, 3);

    EXPECT_NE(spinning_move_intent, spinning_move_intent_other);
}
