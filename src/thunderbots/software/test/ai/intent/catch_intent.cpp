/**
 * This file contains unit tests for the Catch Intent class
 */

#include "ai/intent/catch_intent.h"

#include <gtest/gtest.h>
#include <string.h>

TEST(CatchIntentTest, intent_name_test)
{
    CatchIntent catch_intent = CatchIntent(0, 0, 0, 0, 0);

    EXPECT_EQ("Catch Intent", catch_intent.getIntentName());
}

// For equality operators, we only check for cases not covered in the Primitive tests,
// since Intents inherit from Primitives
TEST(CatchIntentTest, test_equality_operator_intents_equal)
{
    CatchIntent catch_intent       = CatchIntent(0, 0, 0, 0, 0);
    CatchIntent catch_intent_other = CatchIntent(0, 0, 0, 0, 0);

    EXPECT_EQ(catch_intent, catch_intent_other);
}

TEST(CatchIntentTest, test_inequality_operator_with_mismatched_priorities)
{
    CatchIntent catch_intent       = CatchIntent(0, 0, 0, 0, 0);
    CatchIntent catch_intent_other = CatchIntent(0, 0, 0, 0, 1);

    EXPECT_NE(catch_intent, catch_intent_other);
}
