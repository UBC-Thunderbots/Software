/**
 * This file contains unit tests for the Stop Intent class
 */

#include "ai/intent/stop_intent.h"

#include <gtest/gtest.h>
#include <string.h>

TEST(StopIntentTest, intent_name_test)
{
    StopIntent stop_intent = StopIntent(0, false, 0);

    EXPECT_EQ("Stop Intent", stop_intent.getIntentName());
}

// For equality operators, we only check for cases not covered in the Primitive tests,
// since Intents inherit from Primitives
TEST(StopIntentTest, test_equality_operator_intents_equal)
{
    StopIntent stop_intent       = StopIntent(0, false, 0);
    StopIntent stop_intent_other = StopIntent(0, false, 0);

    EXPECT_EQ(stop_intent, stop_intent_other);
}

TEST(StopIntentTest, test_inequality_operator_with_mismatched_priorities)
{
    StopIntent stop_intent       = StopIntent(0, false, 1);
    StopIntent stop_intent_other = StopIntent(0, false, 3);

    EXPECT_NE(stop_intent, stop_intent_other);
}
