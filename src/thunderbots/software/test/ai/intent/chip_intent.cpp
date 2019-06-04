/**
 * This file contains unit tests for the Chip Intent class
 */

#include "ai/intent/chip_intent.h"

#include <gtest/gtest.h>
#include <string.h>

TEST(ChipIntentTest, intent_name_test)
{
    ChipIntent chip_intent = ChipIntent(0, Point(), Angle::zero(), 0, 0);

    EXPECT_EQ("Chip Intent", chip_intent.getIntentName());
}

// For equality operators, we only check for cases not covered in the Primitive tests,
// since Intents inherit from Primitives
TEST(ChipIntentTest, test_equality_operator_intents_equal)
{
    ChipIntent chip_intent       = ChipIntent(0, Point(), Angle::zero(), 0, 0);
    ChipIntent chip_intent_other = ChipIntent(0, Point(), Angle::zero(), 0, 0);

    EXPECT_EQ(chip_intent, chip_intent_other);
}

TEST(ChipIntentTest, test_inequality_operator_with_mismatched_priorities)
{
    ChipIntent chip_intent       = ChipIntent(0, Point(), Angle::zero(), 0, 1);
    ChipIntent chip_intent_other = ChipIntent(0, Point(), Angle::zero(), 0, 3);

    EXPECT_NE(chip_intent, chip_intent_other);
}
