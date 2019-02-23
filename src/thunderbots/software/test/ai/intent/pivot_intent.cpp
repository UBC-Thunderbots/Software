/**
 * This file contains unit tests for the Pivot Intent class
 */

#include "ai/intent/pivot_intent.h"

#include <gtest/gtest.h>
#include <string.h>

TEST(PivotIntentTest, intent_name_test)
{
    PivotIntent pivot_intent = PivotIntent(0, Point(), Angle::zero(), 0, 0);

    EXPECT_EQ("Pivot Intent", pivot_intent.getIntentName());
}

// For equality operators, we only check for cases not covered in the Primitive tests,
// since Intents inherit from Primitives
TEST(PivotIntentTest, test_equality_operator_intents_equal)
{
    PivotIntent pivot_intent       = PivotIntent(0, Point(), Angle::zero(), 0, 0);
    PivotIntent pivot_intent_other = PivotIntent(0, Point(), Angle::zero(), 0, 0);

    EXPECT_EQ(pivot_intent, pivot_intent_other);
}

TEST(PivotIntentTest, test_inequality_operator_with_mismatched_priorities)
{
    PivotIntent pivot_intent       = PivotIntent(0, Point(), Angle::zero(), 0, 1);
    PivotIntent pivot_intent_other = PivotIntent(0, Point(), Angle::zero(), 0, 3);

    EXPECT_NE(pivot_intent, pivot_intent_other);
}
