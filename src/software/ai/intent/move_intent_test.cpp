#include "software/ai/intent/move_intent.h"

#include <gtest/gtest.h>
#include <string.h>

// For equality operators, we only check for cases not covered in the Primitive tests,
// since Intents inherit from Primitives
TEST(MoveIntentTest, test_equality_operator_intents_equal)
{
    MoveIntent move_intent = MoveIntent(0, Point(), Angle::zero(), 0, DribblerMode::OFF,
                                        BallCollisionType::AVOID);
    MoveIntent move_intent_other = MoveIntent(
        0, Point(), Angle::zero(), 0, DribblerMode::OFF, BallCollisionType::AVOID);

    EXPECT_EQ(move_intent, move_intent_other);
}

TEST(MoveIntentTest, test_get_destination_ball_collision)
{
    MoveIntent move_intent = MoveIntent(0, Point(1, 2), Angle::quarter(), 2.3,
                                        DribblerMode::OFF, BallCollisionType::AVOID);
    EXPECT_EQ(move_intent.getDestination(), Point(1, 2));
    EXPECT_EQ(move_intent.getBallCollisionType(), BallCollisionType::AVOID);
}
