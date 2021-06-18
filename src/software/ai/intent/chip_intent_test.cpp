#include "software/ai/intent/chip_intent.h"

#include <gtest/gtest.h>
#include <string.h>

#include "shared/2015_robot_constants.h"

// For equality operators, we only check for cases not covered in the Primitive tests,
// since Intents inherit from Primitives
TEST(ChipIntentTest, test_equality_operator_intents_equal)
{
    RobotConstants_t robot_constants = create2015RobotConstants();
    ChipIntent chip_intent = ChipIntent(0, Point(), Angle::zero(), 0, robot_constants);
    ChipIntent chip_intent_other =
        ChipIntent(0, Point(), Angle::zero(), 0, robot_constants);

    EXPECT_EQ(chip_intent, chip_intent_other);
}
