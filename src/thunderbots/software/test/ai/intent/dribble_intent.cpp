/**
 * This file contains unit tests for the Dribble Intent class
 */

#include "ai/intent/dribble_intent.h"

#include <gtest/gtest.h>
#include <string.h>

TEST(DribbleIntentTest, intent_name_test)
{
    DribbleIntent dribble_intent = DribbleIntent(0, Point(), Angle::zero(), 0, false, 0);

    EXPECT_EQ("Dribble Intent", dribble_intent.getIntentName());
}

// For equality operators, we only check for cases not covered in the Primitive tests,
// since Intents inherit from Primitives
TEST(DribbleIntentTest, test_equality_operator_intents_equal)
{
    DribbleIntent dribble_intent = DribbleIntent(0, Point(), Angle::zero(), 0, false, 0);
    DribbleIntent dribble_intent_other =
        DribbleIntent(0, Point(), Angle::zero(), 0, false, 0);

    EXPECT_EQ(dribble_intent, dribble_intent_other);
}

TEST(DribbleIntentTest, test_inequality_operator_with_mismatched_priorities)
{
    DribbleIntent dribble_intent = DribbleIntent(0, Point(), Angle::zero(), 1, false, 0);
    DribbleIntent dribble_intent_other =
        DribbleIntent(0, Point(), Angle::zero(), 1, false, 4);

    EXPECT_NE(dribble_intent, dribble_intent_other);
}
