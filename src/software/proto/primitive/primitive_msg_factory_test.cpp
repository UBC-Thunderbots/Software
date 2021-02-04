#include "software/proto/primitive/primitive_msg_factory.h"

#include <gtest/gtest.h>

#include "shared/constants.h"

TEST(PrimitiveFactoryTest, test_create_chip_primitive)
{
    auto chip_primitive = createChipPrimitive(Point(1, 2), Angle::quarter(), 5.0);

    ASSERT_TRUE(chip_primitive->has_chip());
    EXPECT_EQ(chip_primitive->chip().chip_origin().x_meters(), 1);
    EXPECT_EQ(chip_primitive->chip().chip_origin().y_meters(), 2);
    EXPECT_EQ(chip_primitive->chip().chip_direction().radians(),
              static_cast<float>(Angle::quarter().toRadians()));
    EXPECT_EQ(chip_primitive->chip().chip_distance_meters(), 5.0);
}

TEST(PrimitiveFactoryTest, test_create_kick_primitive)
{
    auto kick_primitive = createKickPrimitive(Point(3, 5), Angle::half(), 6.5);

    ASSERT_TRUE(kick_primitive->has_kick());
    EXPECT_EQ(kick_primitive->kick().kick_origin().x_meters(), 3);
    EXPECT_EQ(kick_primitive->kick().kick_origin().y_meters(), 5);
    EXPECT_EQ(kick_primitive->kick().kick_direction().radians(),
              static_cast<float>(Angle::half().toRadians()));
    EXPECT_EQ(kick_primitive->kick().kick_speed_meters_per_second(), 6.5);
}

TEST(PrimitiveFactoryTest, test_create_move_primitive)
{
    auto move_primitive = createMovePrimitive(Point(-5, 1), 3.0, Angle::threeQuarter(),
                                              DribblerMode::INDEFINITE);

    ASSERT_TRUE(move_primitive->has_move());
    EXPECT_EQ(move_primitive->move().position_params().destination().x_meters(), -5);
    EXPECT_EQ(move_primitive->move().position_params().destination().y_meters(), 1);
    EXPECT_EQ(move_primitive->move().position_params().final_speed_meters_per_second(),
              3.0);
    EXPECT_EQ(move_primitive->move().final_angle().radians(),
              static_cast<float>(Angle::threeQuarter().toRadians()));
    EXPECT_EQ(move_primitive->move().dribbler_speed_rpm(), INDEFINITE_DRIBBLER_SPEED);
}

TEST(PrimitiveFactoryTest, test_create_spinning_move_primitive)
{
    auto spinning_move_primitive = createSpinningMovePrimitive(
        Point(1, -8), -6.0, AngularVelocity::full(), DribblerMode::OFF);

    ASSERT_TRUE(spinning_move_primitive->has_spinning_move());
    EXPECT_EQ(spinning_move_primitive->spinning_move()
                  .position_params()
                  .destination()
                  .x_meters(),
              1);
    EXPECT_EQ(spinning_move_primitive->spinning_move()
                  .position_params()
                  .destination()
                  .y_meters(),
              -8);
    EXPECT_EQ(spinning_move_primitive->spinning_move()
                  .position_params()
                  .final_speed_meters_per_second(),
              -6.0);
    EXPECT_EQ(
        spinning_move_primitive->spinning_move().angular_velocity().radians_per_second(),
        static_cast<float>(AngularVelocity::full().toRadians()));
    EXPECT_EQ(spinning_move_primitive->spinning_move().dribbler_speed_rpm(), 0.0);
}

TEST(PrimitiveFactoryTest, test_create_autochip_move_primitive)
{
    auto autochip_move_primitive = createAutochipMovePrimitive(
        Point(-4, -10), 6.0, Angle::quarter(), DribblerMode::MAX_FORCE, 2.5);

    ASSERT_TRUE(autochip_move_primitive->has_autochip_move());
    EXPECT_EQ(autochip_move_primitive->autochip_move()
                  .position_params()
                  .destination()
                  .x_meters(),
              -4);
    EXPECT_EQ(autochip_move_primitive->autochip_move()
                  .position_params()
                  .destination()
                  .y_meters(),
              -10);
    EXPECT_EQ(autochip_move_primitive->autochip_move()
                  .position_params()
                  .final_speed_meters_per_second(),
              6.0);
    EXPECT_EQ(autochip_move_primitive->autochip_move().final_angle().radians(),
              static_cast<float>(Angle::quarter().toRadians()));
    EXPECT_EQ(autochip_move_primitive->autochip_move().dribbler_speed_rpm(),
              MAX_FORCE_DRIBBLER_SPEED);
    EXPECT_EQ(autochip_move_primitive->autochip_move().chip_distance_meters(), 2.5);
}

TEST(PrimitiveFactoryTest, test_create_autokick_move_primitive)
{
    auto autokick_move_primitive = createAutokickMovePrimitive(
        Point(5, 12), -2.0, Angle::half(), DribblerMode::INDEFINITE, 2.0);

    ASSERT_TRUE(autokick_move_primitive->has_autokick_move());
    EXPECT_EQ(autokick_move_primitive->autokick_move()
                  .position_params()
                  .destination()
                  .x_meters(),
              5);
    EXPECT_EQ(autokick_move_primitive->autokick_move()
                  .position_params()
                  .destination()
                  .y_meters(),
              12);
    EXPECT_EQ(autokick_move_primitive->autokick_move()
                  .position_params()
                  .final_speed_meters_per_second(),
              -2.0);
    EXPECT_EQ(autokick_move_primitive->autokick_move().final_angle().radians(),
              static_cast<float>(Angle::half().toRadians()));
    EXPECT_EQ(autokick_move_primitive->autokick_move().dribbler_speed_rpm(),
              INDEFINITE_DRIBBLER_SPEED);
    EXPECT_EQ(autokick_move_primitive->autokick_move().kick_speed_meters_per_second(),
              2.0);
}

TEST(PrimitiveFactoryTest, test_create_stop_primitive_brake)
{
    auto stop_primitive = createStopPrimitive(false);

    ASSERT_TRUE(stop_primitive->has_stop());
    EXPECT_EQ(stop_primitive->stop().stop_type(), TbotsProto::StopPrimitive::BRAKE);
}

TEST(PrimitiveFactoryTest, test_create_stop_primitive_coast)
{
    auto stop_primitive = createStopPrimitive(true);

    ASSERT_TRUE(stop_primitive->has_stop());
    EXPECT_EQ(stop_primitive->stop().stop_type(), TbotsProto::StopPrimitive::COAST);
}
