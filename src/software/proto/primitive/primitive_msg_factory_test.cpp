#include "software/proto/primitive/primitive_msg_factory.h"

#include <gtest/gtest.h>

TEST(PrimitiveMsgFactoryTest, test_create_chip_primitive)
{
    auto chip_primitive = createChipPrimitiveMsg(Point(1, 2), Angle::quarter(), 5.0);

    EXPECT_EQ(chip_primitive->mutable_chip()->chip_origin().x_meters(), 1);
    EXPECT_EQ(chip_primitive->mutable_chip()->chip_origin().y_meters(), 2);
    EXPECT_EQ(chip_primitive->mutable_chip()->chip_direction().radians(),
              static_cast<float>(Angle::quarter().toRadians()));
    EXPECT_EQ(chip_primitive->mutable_chip()->chip_distance_meters(), 5.0);
}

TEST(PrimitiveMsgFactoryTest, test_create_kick_primitive)
{
    auto kick_primitive = createKickPrimitiveMsg(Point(3, 5), Angle::half(), 6.5);

    EXPECT_EQ(kick_primitive->mutable_kick()->kick_origin().x_meters(), 3);
    EXPECT_EQ(kick_primitive->mutable_kick()->kick_origin().y_meters(), 5);
    EXPECT_EQ(kick_primitive->mutable_kick()->kick_direction().radians(),
              static_cast<float>(Angle::half().toRadians()));
    EXPECT_EQ(kick_primitive->mutable_kick()->kick_speed_meters_per_second(), 6.5);
}

TEST(PrimitiveMsgFactoryTest, test_create_move_primitive)
{
    auto move_primitive =
        createMovePrimitiveMsg(Point(-5, 1), 3.0, true, Angle::threeQuarter(), 2.0);

    EXPECT_EQ(move_primitive->mutable_move()->position_params().destination().x_meters(),
              -5);
    EXPECT_EQ(move_primitive->mutable_move()->position_params().destination().y_meters(),
              1);
    EXPECT_EQ(
        move_primitive->mutable_move()->position_params().final_speed_meters_per_second(),
        3.0);
    EXPECT_TRUE(move_primitive->mutable_move()->position_params().slow());
    EXPECT_EQ(move_primitive->mutable_move()->final_angle().radians(),
              static_cast<float>(Angle::threeQuarter().toRadians()));
    EXPECT_EQ(move_primitive->mutable_move()->dribbler_speed_rpm(), 2.0);
}

TEST(PrimitiveMsgFactoryTest, test_create_spinning_move_primitive)
{
    auto spinning_move_primitive = createSpinningMovePrimitiveMsg(
        Point(1, -8), -6.0, false, AngularVelocity::full(), -1.0);

    EXPECT_EQ(spinning_move_primitive->mutable_spinning_move()
                  ->position_params()
                  .destination()
                  .x_meters(),
              1);
    EXPECT_EQ(spinning_move_primitive->mutable_spinning_move()
                  ->position_params()
                  .destination()
                  .y_meters(),
              -8);
    EXPECT_EQ(spinning_move_primitive->mutable_spinning_move()
                  ->position_params()
                  .final_speed_meters_per_second(),
              -6.0);
    EXPECT_FALSE(
        spinning_move_primitive->mutable_spinning_move()->position_params().slow());
    EXPECT_EQ(spinning_move_primitive->mutable_spinning_move()
                  ->angular_velocity()
                  .radians_per_second(),
              static_cast<float>(AngularVelocity::full().toRadians()));
    EXPECT_EQ(spinning_move_primitive->mutable_spinning_move()->dribbler_speed_rpm(),
              -1.0);
}

TEST(PrimitiveMsgFactoryTest, test_create_autochip_move_primitive)
{
    auto autochip_move_primitive = createAutochipMovePrimitiveMsg(
        Point(-4, -10), 6.0, true, Angle::quarter(), 1.0, 2.5);

    EXPECT_EQ(autochip_move_primitive->mutable_autochip_move()
                  ->position_params()
                  .destination()
                  .x_meters(),
              -4);
    EXPECT_EQ(autochip_move_primitive->mutable_autochip_move()
                  ->position_params()
                  .destination()
                  .y_meters(),
              -10);
    EXPECT_EQ(autochip_move_primitive->mutable_autochip_move()
                  ->position_params()
                  .final_speed_meters_per_second(),
              6.0);
    EXPECT_TRUE(
        autochip_move_primitive->mutable_autochip_move()->position_params().slow());
    EXPECT_EQ(autochip_move_primitive->mutable_autochip_move()->final_angle().radians(),
              static_cast<float>(Angle::quarter().toRadians()));
    EXPECT_EQ(autochip_move_primitive->mutable_autochip_move()->dribbler_speed_rpm(),
              1.0);
    EXPECT_EQ(autochip_move_primitive->mutable_autochip_move()->chip_distance_meters(),
              2.5);
}

TEST(PrimitiveMsgFactoryTest, test_create_autokick_move_primitive)
{
    auto autokick_move_primitive = createAutokickMovePrimitiveMsg(
        Point(5, 12), -2.0, false, Angle::half(), 5.5, 2.0);

    EXPECT_EQ(autokick_move_primitive->mutable_autokick_move()
                  ->position_params()
                  .destination()
                  .x_meters(),
              5);
    EXPECT_EQ(autokick_move_primitive->mutable_autokick_move()
                  ->position_params()
                  .destination()
                  .y_meters(),
              12);
    EXPECT_EQ(autokick_move_primitive->mutable_autokick_move()
                  ->position_params()
                  .final_speed_meters_per_second(),
              -2.0);
    EXPECT_FALSE(
        autokick_move_primitive->mutable_autokick_move()->position_params().slow());
    EXPECT_EQ(autokick_move_primitive->mutable_autokick_move()->final_angle().radians(),
              static_cast<float>(Angle::half().toRadians()));
    EXPECT_EQ(autokick_move_primitive->mutable_autokick_move()->dribbler_speed_rpm(),
              5.5);
    EXPECT_EQ(
        autokick_move_primitive->mutable_autokick_move()->kick_speed_meters_per_second(),
        2.0);
}

TEST(PrimitiveMsgFactoryTest, test_create_stop_primitive_brake)
{
    auto stop_primitive = createStopPrimitiveMsg(StopType::BRAKE);

    EXPECT_EQ(stop_primitive->mutable_stop()->stop_type(),
              stop_primitive->mutable_stop()->BRAKE);
}

TEST(PrimitiveMsgFactoryTest, test_create_stop_primitive_coast)
{
    auto stop_primitive = createStopPrimitiveMsg(StopType::COAST);

    EXPECT_EQ(stop_primitive->mutable_stop()->stop_type(),
              stop_primitive->mutable_stop()->COAST);
}
