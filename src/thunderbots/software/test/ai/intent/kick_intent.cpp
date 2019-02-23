/**
 * This file contains unit tests for the Kick Intent class
 */

#include "ai/intent/kick_intent.h"

#include <gtest/gtest.h>
#include <string.h>

TEST(KickIntentTest, intent_name_test)
{
    KickIntent kick_intent = KickIntent(0, Point(), Angle::zero(), 0, 0);

    EXPECT_EQ("Kick Intent", kick_intent.getIntentName());
}

// For equality operators, we only check for cases not covered in the Primitive tests,
// since Intents inherit from Primitives
TEST(KickIntentTest, test_equality_operator_intents_equal)
{
    KickIntent kick_intent       = KickIntent(0, Point(), Angle::zero(), 0, 0);
    KickIntent kick_intent_other = KickIntent(0, Point(), Angle::zero(), 0, 0);

    EXPECT_EQ(kick_intent, kick_intent_other);
}

TEST(KickIntentTest, test_inequality_operator_with_mismatched_priorities)
{
    KickIntent kick_intent       = KickIntent(0, Point(), Angle::zero(), 0, 9);
    KickIntent kick_intent_other = KickIntent(0, Point(), Angle::zero(), 0, 7);

    EXPECT_NE(kick_intent, kick_intent_other);
}
