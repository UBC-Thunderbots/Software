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

//// For equality operators, we only check for cases not covered in the Primitive tests,
//// since Intents inherit from Primitives
// TEST(AutochipMoveIntentTest, test_equality_operator_intents_equal)
//{
//    AutochipMoveIntent intent = AutochipMoveIntent(
//        0, Point(), Angle::zero(), 0, DribblerMode::OFF, 2.2, BallCollisionType::AVOID);
//    AutochipMoveIntent intent_other = AutochipMoveIntent(
//        0, Point(), Angle::zero(), 0, DribblerMode::OFF, 2.2, BallCollisionType::AVOID);
//
//    EXPECT_EQ(intent, intent_other);
//}
//
// TEST(AutochipMoveIntentTest, test_get_destination_ball_collision)
//{
//    AutochipMoveIntent intent =
//        AutochipMoveIntent(0, Point(1, 2), Angle::quarter(), 2.3,
//        DribblerMode::OFF, 3.2,
//                           BallCollisionType::AVOID);
//    EXPECT_EQ(intent.getDestination(), Point(1, 2));
//    EXPECT_EQ(intent.getChipDistance(), 3.2);
//    EXPECT_EQ(intent.getFinalAngle(), Angle::quarter());
//    EXPECT_EQ(intent.getDribblerMode(), DribblerMode::OFF);
//    EXPECT_EQ(intent.getBallCollisionType(), BallCollisionType::AVOID);
//}
//
// TEST(AutochipMoveIntentTest, test_intent_pointer)
//{
//    std::shared_ptr<Intent> intent = std::make_shared<AutochipMoveIntent>(
//        AutochipMoveIntent(0, Point(1, 2), Angle::quarter(), 2.3,
//        DribblerMode::OFF, 3.2,
//                           BallCollisionType::AVOID));
//    EXPECT_EQ("AutochipMoveIntent", TYPENAME(*intent));
//}
//
//// For equality operators, we only check for cases not covered in the Primitive tests,
//// since Intents inherit from Primitives
// TEST(AutokickMoveIntentTest, test_equality_operator_intents_equal)
//{
//    AutokickMoveIntent intent = AutokickMoveIntent(
//        0, Point(), Angle::zero(), 0, DribblerMode::OFF, 2.2, BallCollisionType::AVOID);
//    AutokickMoveIntent intent_other = AutokickMoveIntent(
//        0, Point(), Angle::zero(), 0, DribblerMode::OFF, 2.2, BallCollisionType::AVOID);
//
//    EXPECT_EQ(intent, intent_other);
//}
//
// TEST(AutokickMoveIntentTest, test_get_destination_ball_collision)
//{
//    AutokickMoveIntent intent =
//        AutokickMoveIntent(0, Point(1, 2), Angle::quarter(), 2.3,
//        DribblerMode::OFF, 1.2,
//                           BallCollisionType::AVOID);
//    EXPECT_EQ(intent.getDestination(), Point(1, 2));
//    EXPECT_EQ(intent.getKickSpeed(), 1.2);
//    EXPECT_EQ(intent.getFinalAngle(), Angle::quarter());
//    EXPECT_EQ(intent.getDribblerMode(), DribblerMode::OFF);
//    EXPECT_EQ(intent.getBallCollisionType(), BallCollisionType::AVOID);
//}
//
// TEST(AutokickMoveIntentTest, test_intent_pointer)
//{
//    std::shared_ptr<Intent> intent = std::make_shared<AutokickMoveIntent>(
//        AutokickMoveIntent(0, Point(1, 2), Angle::quarter(), 2.3,
//        DribblerMode::OFF, 3.2,
//                           BallCollisionType::AVOID));
//    EXPECT_EQ("AutokickMoveIntent", TYPENAME(*intent));
//}
