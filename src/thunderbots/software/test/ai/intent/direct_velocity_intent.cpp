/**
 * This file contains unit tests for the DirectVelocity Intent class
 */

#include "ai/intent/direct_velocity_intent.h"

#include <gtest/gtest.h>
#include <string.h>

TEST(DirectVelocityIntentTest, intent_name_test)
{
    DirectVelocityIntent direct_velocity_intent = DirectVelocityIntent(0, 0, 0, 0, 0, 0);

    EXPECT_EQ("Direct Velocity Intent", direct_velocity_intent.getIntentName());
}

// For equality operators, we only check for cases not covered in the Primitive tests,
// since Intents inherit from Primitives
TEST(DirectVelocityIntentTest, test_equality_operator_intents_equal)
{
    DirectVelocityIntent direct_velocity_intent = DirectVelocityIntent(0, 0, 0, 0, 0, 0);
    DirectVelocityIntent direct_velocity_intent_other =
        DirectVelocityIntent(0, 0, 0, 0, 0, 0);

    EXPECT_EQ(direct_velocity_intent, direct_velocity_intent_other);
}

TEST(DirectVelocityIntentTest, test_inequality_operator_with_mismatched_priorities)
{
    DirectVelocityIntent direct_velocity_intent = DirectVelocityIntent(0, 0, 0, 0, 0, 0);
    DirectVelocityIntent direct_velocity_intent_other =
        DirectVelocityIntent(0, 0, 0, 0, 0, 2);

    EXPECT_NE(direct_velocity_intent, direct_velocity_intent_other);
}
