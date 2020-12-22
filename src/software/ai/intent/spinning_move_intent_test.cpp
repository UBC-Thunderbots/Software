#include "software/ai/intent/spinning_move_intent.h"

#include <gtest/gtest.h>
#include <string.h>

// For equality operators, we only check for cases not covered in the Primitive tests,
// since Intents inherit from Primitives
TEST(SpinningMoveIntentTest, test_equality_operator_intents_equal)
{
    SpinningMoveIntent spinning_move_intent =
        SpinningMoveIntent(0, Point(), Angle::zero(), 1.0);
    SpinningMoveIntent spinning_move_intent_other =
        SpinningMoveIntent(0, Point(), Angle::zero(), 1.0);

    EXPECT_EQ(spinning_move_intent, spinning_move_intent_other);
}
