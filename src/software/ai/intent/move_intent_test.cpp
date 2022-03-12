#include "software/ai/intent/move_intent.h"

#include <gtest/gtest.h>
#include <string.h>

#include "proto/primitive/primitive_msg_factory.h"
#include "shared/2015_robot_constants.h"
#include "software/util/typename/typename.h"

class MoveIntentTest : public ::testing::Test
{
   protected:
    RobotConstants_t robot_constants = create2015RobotConstants();
};

class AutochipMoveIntentTest : public MoveIntentTest
{
};

// For equality operators, we only check for cases not covered in the Primitive tests,
// since Intents inherit from Primitives
TEST_F(MoveIntentTest, test_equality_operator_intents_equal)
{
    MoveIntent move_intent =
        MoveIntent(0, Point(), Angle::zero(), 0, TbotsProto::DribblerMode::OFF,
                   TbotsProto::BallCollisionType::AVOID, {AutoChipOrKickMode::OFF, 2},
                   TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0, robot_constants);
    MoveIntent move_intent_other =
        MoveIntent(0, Point(), Angle::zero(), 0, TbotsProto::DribblerMode::OFF,
                   TbotsProto::BallCollisionType::AVOID, {AutoChipOrKickMode::OFF, 4},
                   TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0, robot_constants);
    EXPECT_EQ(move_intent, move_intent_other);

    move_intent = MoveIntent(
        0, Point(), Angle::zero(), 0, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::AVOID, {AutoChipOrKickMode::AUTOKICK, 3},
        TbotsProto::MaxAllowedSpeedMode::STOP_COMMAND, -2.0, robot_constants);
    move_intent_other = MoveIntent(
        0, Point(), Angle::zero(), 0, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::AVOID, {AutoChipOrKickMode::AUTOKICK, 3},
        TbotsProto::MaxAllowedSpeedMode::STOP_COMMAND, -2.0, robot_constants);
    EXPECT_EQ(move_intent, move_intent_other);
}

TEST_F(MoveIntentTest, test_equality_operator_intents_not_equal)
{
    MoveIntent move_intent = MoveIntent(
        0, Point(), Angle::zero(), 0, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::AVOID, {AutoChipOrKickMode::AUTOCHIP, 1.5},
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 6.0, robot_constants);
    MoveIntent move_intent_other = MoveIntent(
        0, Point(), Angle::zero(), 0, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::AVOID, {AutoChipOrKickMode::AUTOCHIP, 3.5},
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 6.0, robot_constants);
    EXPECT_NE(move_intent, move_intent_other);

    move_intent = MoveIntent(
        0, Point(), Angle::zero(), 0, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::AVOID, {AutoChipOrKickMode::AUTOCHIP, 20},
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0, robot_constants);
    move_intent_other = MoveIntent(
        0, Point(), Angle::zero(), 0, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::AVOID, {AutoChipOrKickMode::AUTOCHIP, 20},
        TbotsProto::MaxAllowedSpeedMode::STOP_COMMAND, 0.0, robot_constants);
    EXPECT_NE(move_intent, move_intent_other);

    move_intent = MoveIntent(
        0, Point(), Angle::zero(), 0, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::AVOID, {AutoChipOrKickMode::AUTOCHIP, 20},
        TbotsProto::MaxAllowedSpeedMode::STOP_COMMAND, 0.0, robot_constants);
    move_intent_other = MoveIntent(
        0, Point(), Angle::zero(), 0, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::AVOID, {AutoChipOrKickMode::AUTOCHIP, 20},
        TbotsProto::MaxAllowedSpeedMode::STOP_COMMAND, 7.0, robot_constants);
    EXPECT_NE(move_intent, move_intent_other);
}

TEST_F(MoveIntentTest, test_get_destination_ball_collision)
{
    MoveIntent move_intent =
        MoveIntent(0, Point(1, 2), Angle::quarter(), 2.3, TbotsProto::DribblerMode::OFF,
                   TbotsProto::BallCollisionType::AVOID, {AutoChipOrKickMode::OFF, 0},
                   TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0, robot_constants);
    EXPECT_EQ(move_intent.getDestination(), Point(1, 2));
    EXPECT_EQ(move_intent.getBallCollisionType(), TbotsProto::BallCollisionType::AVOID);
}

TEST_F(AutochipMoveIntentTest, test_get_destination_ball_collision_chip)
{
    MoveIntent intent = MoveIntent(
        0, Point(1, 2), Angle::quarter(), 2.3, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::AVOID, {AutoChipOrKickMode::AUTOCHIP, 3.2},
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0, robot_constants);
    EXPECT_EQ(intent.getDestination(), Point(1, 2));
    ASSERT_TRUE(intent.getAutoChipOrKick().auto_chip_kick_mode ==
                AutoChipOrKickMode::AUTOCHIP);
    EXPECT_EQ(intent.getAutoChipOrKick().autochip_distance_m, 3.2);
    EXPECT_EQ(intent.getFinalAngle(), Angle::quarter());
    EXPECT_EQ(intent.getDribblerMode(), TbotsProto::DribblerMode::OFF);
    EXPECT_EQ(intent.getBallCollisionType(), TbotsProto::BallCollisionType::AVOID);
    EXPECT_EQ(intent.getMaxAllowedSpeedMode(),
              TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
    EXPECT_EQ(intent.getTargetSpinRevPerS(), 0.0);
}

TEST_F(AutochipMoveIntentTest, test_intent_pointer)
{
    std::shared_ptr<Intent> intent = std::make_shared<MoveIntent>(MoveIntent(
        0, Point(1, 2), Angle::quarter(), 2.3, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::AVOID, {AutoChipOrKickMode::AUTOCHIP, 3.2},
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0, robot_constants));
    EXPECT_EQ("MoveIntent", objectTypeName(*intent));
}

TEST_F(MoveIntentTest, test_get_destination_ball_collision_kick_stop_command_spin_speed)
{
    MoveIntent intent = MoveIntent(
        0, Point(1, 2), Angle::quarter(), 2.3, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::AVOID, {AutoChipOrKickMode::AUTOKICK, 3.2},
        TbotsProto::MaxAllowedSpeedMode::STOP_COMMAND, 4.5, robot_constants);
    EXPECT_EQ(intent.getDestination(), Point(1, 2));
    ASSERT_EQ(intent.getAutoChipOrKick().auto_chip_kick_mode,
              AutoChipOrKickMode::AUTOKICK);
    EXPECT_EQ(intent.getAutoChipOrKick().autokick_speed_m_per_s, 3.2);
    EXPECT_EQ(intent.getFinalAngle(), Angle::quarter());
    EXPECT_EQ(intent.getDribblerMode(), TbotsProto::DribblerMode::OFF);
    EXPECT_EQ(intent.getBallCollisionType(), TbotsProto::BallCollisionType::AVOID);
    EXPECT_EQ(intent.getMaxAllowedSpeedMode(),
              TbotsProto::MaxAllowedSpeedMode::STOP_COMMAND);
    EXPECT_EQ(intent.getTargetSpinRevPerS(), 4.5);
}
