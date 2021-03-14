#include "software/ai/intent/move_intent.h"

#include <gtest/gtest.h>
#include <string.h>

#include "software/proto/primitive/primitive_msg_factory.h"
#include "software/util/typename/typename.h"

// For equality operators, we only check for cases not covered in the Primitive tests,
// since Intents inherit from Primitives
TEST(MoveIntentTest, test_equality_operator_intents_equal)
{
    MoveIntent move_intent =
        MoveIntent(0, Point(), Angle::zero(), 0, DribblerMode::OFF,
                   BallCollisionType::AVOID, {AutoChipOrKickMode::OFF, 2},
                   MaxAllowedSpeedMode::PHYSICAL_LIMIT, AngularVelocity::zero());
    MoveIntent move_intent_other =
        MoveIntent(0, Point(), Angle::zero(), 0, DribblerMode::OFF,
                   BallCollisionType::AVOID, {AutoChipOrKickMode::OFF, 4},
                   MaxAllowedSpeedMode::PHYSICAL_LIMIT, AngularVelocity::zero());
    EXPECT_EQ(move_intent, move_intent_other);

    move_intent =
        MoveIntent(0, Point(), Angle::zero(), 0, DribblerMode::OFF,
                   BallCollisionType::AVOID, {AutoChipOrKickMode::AUTOKICK, 3},
                   MaxAllowedSpeedMode::STOP_COMMAND, AngularVelocity::quarter());
    move_intent_other =
        MoveIntent(0, Point(), Angle::zero(), 0, DribblerMode::OFF,
                   BallCollisionType::AVOID, {AutoChipOrKickMode::AUTOKICK, 3},
                   MaxAllowedSpeedMode::STOP_COMMAND, AngularVelocity::quarter());
    EXPECT_EQ(move_intent, move_intent_other);
}

TEST(MoveIntentTest, test_equality_operator_intents_not_equal)
{
    MoveIntent move_intent =
        MoveIntent(0, Point(), Angle::zero(), 0, DribblerMode::OFF,
                   BallCollisionType::AVOID, {AutoChipOrKickMode::AUTOCHIP, 1.5},
                   MaxAllowedSpeedMode::PHYSICAL_LIMIT, AngularVelocity::half());
    MoveIntent move_intent_other =
        MoveIntent(0, Point(), Angle::zero(), 0, DribblerMode::OFF,
                   BallCollisionType::AVOID, {AutoChipOrKickMode::AUTOCHIP, 3.5},
                   MaxAllowedSpeedMode::PHYSICAL_LIMIT, AngularVelocity::half());
    EXPECT_NE(move_intent, move_intent_other);

    move_intent =
        MoveIntent(0, Point(), Angle::zero(), 0, DribblerMode::OFF,
                   BallCollisionType::AVOID, {AutoChipOrKickMode::AUTOCHIP, 20},
                   MaxAllowedSpeedMode::PHYSICAL_LIMIT, AngularVelocity::zero());
    move_intent_other =
        MoveIntent(0, Point(), Angle::zero(), 0, DribblerMode::OFF,
                   BallCollisionType::AVOID, {AutoChipOrKickMode::AUTOCHIP, 20},
                   MaxAllowedSpeedMode::STOP_COMMAND, AngularVelocity::zero());
    EXPECT_NE(move_intent, move_intent_other);

    move_intent = MoveIntent(0, Point(), Angle::zero(), 0, DribblerMode::OFF,
                             BallCollisionType::AVOID, {AutoChipOrKickMode::AUTOCHIP, 20},
                             MaxAllowedSpeedMode::STOP_COMMAND, AngularVelocity::zero());
    move_intent_other =
        MoveIntent(0, Point(), Angle::zero(), 0, DribblerMode::OFF,
                   BallCollisionType::AVOID, {AutoChipOrKickMode::AUTOCHIP, 20},
                   MaxAllowedSpeedMode::STOP_COMMAND, AngularVelocity::quarter());
    EXPECT_NE(move_intent, move_intent_other);
}

TEST(MoveIntentTest, test_get_destination_ball_collision)
{
    MoveIntent move_intent =
        MoveIntent(0, Point(1, 2), Angle::quarter(), 2.3, DribblerMode::OFF,
                   BallCollisionType::AVOID, {AutoChipOrKickMode::OFF, 0},
                   MaxAllowedSpeedMode::PHYSICAL_LIMIT, AngularVelocity::zero());
    EXPECT_EQ(move_intent.getDestination(), Point(1, 2));
    EXPECT_EQ(move_intent.getBallCollisionType(), BallCollisionType::AVOID);
}

TEST(AutochipMoveIntentTest, test_get_destination_ball_collision_chip)
{
    MoveIntent intent =
        MoveIntent(0, Point(1, 2), Angle::quarter(), 2.3, DribblerMode::OFF,
                   BallCollisionType::AVOID, {AutoChipOrKickMode::AUTOCHIP, 3.2},
                   MaxAllowedSpeedMode::PHYSICAL_LIMIT, AngularVelocity::zero());
    EXPECT_EQ(intent.getDestination(), Point(1, 2));
    ASSERT_TRUE(intent.getAutoChipOrKick().auto_chip_kick_mode ==
                AutoChipOrKickMode::AUTOCHIP);
    EXPECT_EQ(intent.getAutoChipOrKick().autochip_distance_m, 3.2);
    EXPECT_EQ(intent.getFinalAngle(), Angle::quarter());
    EXPECT_EQ(intent.getDribblerMode(), DribblerMode::OFF);
    EXPECT_EQ(intent.getBallCollisionType(), BallCollisionType::AVOID);
}

TEST(AutochipMoveIntentTest, test_intent_pointer)
{
    std::shared_ptr<Intent> intent = std::make_shared<MoveIntent>(
        MoveIntent(0, Point(1, 2), Angle::quarter(), 2.3, DribblerMode::OFF,
                   BallCollisionType::AVOID, {AutoChipOrKickMode::AUTOCHIP, 3.2},
                   MaxAllowedSpeedMode::PHYSICAL_LIMIT, AngularVelocity::zero()));
    EXPECT_EQ("MoveIntent", TYPENAME(*intent));
}

TEST(MoveIntentTest, test_get_destination_ball_collision_kick)
{
    MoveIntent intent =
        MoveIntent(0, Point(1, 2), Angle::quarter(), 2.3, DribblerMode::OFF,
                   BallCollisionType::AVOID, {AutoChipOrKickMode::AUTOKICK, 3.2},
                   MaxAllowedSpeedMode::PHYSICAL_LIMIT, AngularVelocity::zero());
    EXPECT_EQ(intent.getDestination(), Point(1, 2));
    ASSERT_EQ(intent.getAutoChipOrKick().auto_chip_kick_mode,
              AutoChipOrKickMode::AUTOKICK);
    EXPECT_EQ(intent.getAutoChipOrKick().autokick_speed_m_per_s, 3.2);
    EXPECT_EQ(intent.getFinalAngle(), Angle::quarter());
    EXPECT_EQ(intent.getDribblerMode(), DribblerMode::OFF);
    EXPECT_EQ(intent.getBallCollisionType(), BallCollisionType::AVOID);
}
