#include "software/ai/intent/stop_intent.h"

#include <gtest/gtest.h>
#include <string.h>

// For equality operators, we only check for cases not covered in the Primitive tests,
// since Intents inherit from Primitives
TEST(StopIntentTest, test_equality_operator_intents_equal)
{
    StopIntent stop_intent       = StopIntent(0, false);
    StopIntent stop_intent_other = StopIntent(0, false);

    EXPECT_EQ(stop_intent, stop_intent_other);
}
