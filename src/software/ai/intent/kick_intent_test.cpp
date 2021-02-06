#include "software/ai/intent/kick_intent.h"

#include <gtest/gtest.h>
#include <string.h>

// For equality operators, we only check for cases not covered in the Primitive tests,
// since Intents inherit from Primitives
TEST(KickIntentTest, test_equality_operator_intents_equal)
{
    KickIntent kick_intent       = KickIntent(0, Point(), Angle::zero(), 0);
    KickIntent kick_intent_other = KickIntent(0, Point(), Angle::zero(), 0);

    EXPECT_EQ(kick_intent, kick_intent_other);
}
