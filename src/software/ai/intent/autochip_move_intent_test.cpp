#include "software/ai/intent/autochip_move_intent.h"

#include <gtest/gtest.h>
#include <stdio.h>
#include <string.h>

#include "software/util/typename/typename.h"

// For equality operators, we only check for cases not covered in the Primitive tests,
// since Intents inherit from Primitives
TEST(AutochipMoveIntentTest, test_equality_operator_intents_equal)
{
    AutochipMoveIntent intent = AutochipMoveIntent(
        0, Point(), Angle::zero(), 0, DribblerMode::OFF, 2.2, BallCollisionType::AVOID);
    AutochipMoveIntent intent_other = AutochipMoveIntent(
        0, Point(), Angle::zero(), 0, DribblerMode::OFF, 2.2, BallCollisionType::AVOID);

    EXPECT_EQ(intent, intent_other);
}

TEST(AutochipMoveIntentTest, test_get_destination_ball_collision)
{
    AutochipMoveIntent intent =
        AutochipMoveIntent(0, Point(1, 2), Angle::quarter(), 2.3, DribblerMode::OFF, 3.2,
                           BallCollisionType::AVOID);
    EXPECT_EQ(intent.getDestination(), Point(1, 2));
    EXPECT_EQ(intent.getChipDistance(), 3.2);
    EXPECT_EQ(intent.getFinalAngle(), Angle::quarter());
    EXPECT_EQ(intent.getDribblerMode(), DribblerMode::OFF);
    EXPECT_EQ(intent.getBallCollisionType(), BallCollisionType::AVOID);
}

TEST(AutochipMoveIntentTest, test_intent_pointer)
{
    std::shared_ptr<Intent> intent = std::make_shared<AutochipMoveIntent>(
        AutochipMoveIntent(0, Point(1, 2), Angle::quarter(), 2.3, DribblerMode::OFF, 3.2,
                           BallCollisionType::AVOID));
    EXPECT_EQ("AutochipMoveIntent", TYPENAME(*intent));
}
