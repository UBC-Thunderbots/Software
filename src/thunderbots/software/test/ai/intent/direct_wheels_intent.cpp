/**
 * This file contains unit tests for the DirectWheels Intent class
 */

#include "ai/intent/direct_wheels_intent.h"

#include <gtest/gtest.h>
#include <string.h>

TEST(DirectWheelsIntentTest, intent_name_test)
{
    DirectWheelsIntent direct_wheels_intent = DirectWheelsIntent(0, 0, 0, 0, 0, 0, 0);

    EXPECT_EQ("Direct Wheels Intent", direct_wheels_intent.getIntentName());
}

// For equality operators, we only check for cases not covered in the Primitive tests,
// since Intents inherit from Primitives
TEST(DirectWheelsIntentTest, test_equality_operator_intents_equal)
{
    DirectWheelsIntent direct_wheels_intent = DirectWheelsIntent(0, 0, 0, 0, 0, 0, 0);
    DirectWheelsIntent direct_wheels_intent_other =
        DirectWheelsIntent(0, 0, 0, 0, 0, 0, 0);

    EXPECT_EQ(direct_wheels_intent, direct_wheels_intent_other);
}

TEST(DirectWheelsIntentTest, test_inequality_operator_with_mismatched_priorities)
{
    DirectWheelsIntent direct_wheels_intent = DirectWheelsIntent(0, 0, 0, 0, 0, 0, 1);
    DirectWheelsIntent direct_wheels_intent_other =
        DirectWheelsIntent(0, 0, 0, 0, 0, 0, 3);

    EXPECT_NE(direct_wheels_intent, direct_wheels_intent_other);
}
