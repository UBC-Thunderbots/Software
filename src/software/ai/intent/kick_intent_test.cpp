#include "software/ai/intent/kick_intent.h"

#include <gtest/gtest.h>
#include <string.h>

#include "shared/2015_robot_constants.h"

// For equality operators, we only check for cases not covered in the Primitive tests,
// since Intents inherit from Primitives
TEST(KickIntentTest, test_equality_operator_intents_equal)
{
    RobotConstants_t robot_constants = create2015RobotConstants();
    KickIntent kick_intent = KickIntent(0, Point(), Angle::zero(), 0, robot_constants);
    KickIntent kick_intent_other =
        KickIntent(0, Point(), Angle::zero(), 0, robot_constants);

    EXPECT_EQ(kick_intent, kick_intent_other);
}
