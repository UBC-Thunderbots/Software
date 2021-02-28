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
    EXPECT_EQ(kick_primitive->kick().kick_speed_m_per_s(), 6.5);
}

TEST(PrimitiveFactoryTest, test_create_move_primitive)
{
    auto move_primitive = createMovePrimitive(Point(-5, 1), 3.0, Angle::threeQuarter(),
                                              DribblerMode::INDEFINITE, std::nullopt,
                                              MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    ASSERT_TRUE(move_primitive->has_move());
    EXPECT_EQ(move_primitive->move().destination().x_meters(), -5);
    EXPECT_EQ(move_primitive->move().destination().y_meters(), 1);
    EXPECT_EQ(move_primitive->move().final_speed_m_per_s(), 3.0);
    EXPECT_EQ(move_primitive->move().final_angle().radians(),
              static_cast<float>(Angle::threeQuarter().toRadians()));
    EXPECT_EQ(move_primitive->move().dribbler_speed_rpm(), INDEFINITE_DRIBBLER_SPEED);
    EXPECT_FALSE(move_primitive->move().has_auto_chip_or_kick());
    EXPECT_EQ(move_primitive->move().max_speed_m_per_s(),
              ROBOT_MAX_SPEED_METERS_PER_SECOND);
}

TEST(PrimitiveFactoryTest, test_create_move_primitive_with_autochip)
{
    auto move_primitive = createMovePrimitive(
        Point(-5, 1), 3.0, Angle::threeQuarter(), DribblerMode::INDEFINITE,
        createAutoChipCommand(2.5), MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    ASSERT_TRUE(move_primitive->has_move());
    EXPECT_EQ(move_primitive->move().destination().x_meters(), -5);
    EXPECT_EQ(move_primitive->move().destination().y_meters(), 1);
    EXPECT_EQ(move_primitive->move().final_speed_m_per_s(), 3.0);
    EXPECT_EQ(move_primitive->move().final_angle().radians(),
              static_cast<float>(Angle::threeQuarter().toRadians()));
    EXPECT_EQ(move_primitive->move().dribbler_speed_rpm(), INDEFINITE_DRIBBLER_SPEED);
    ASSERT_TRUE(move_primitive->move().has_auto_chip_or_kick());
    EXPECT_EQ(move_primitive->move().auto_chip_or_kick().autochip_distance_meters(), 2.5);
    EXPECT_EQ(move_primitive->move().max_speed_m_per_s(),
              ROBOT_MAX_SPEED_METERS_PER_SECOND);
}

TEST(PrimitiveFactoryTest, test_create_move_primitive_with_autokick)
{
    auto move_primitive = createMovePrimitive(
        Point(-5, 1), 3.0, Angle::threeQuarter(), DribblerMode::INDEFINITE,
        createAutoKickCommand(3.5), MaxAllowedSpeedMode::STOP_COMMAND);

    ASSERT_TRUE(move_primitive->has_move());
    EXPECT_EQ(move_primitive->move().destination().x_meters(), -5);
    EXPECT_EQ(move_primitive->move().destination().y_meters(), 1);
    EXPECT_EQ(move_primitive->move().final_speed_m_per_s(), 3.0);
    EXPECT_EQ(move_primitive->move().final_angle().radians(),
              static_cast<float>(Angle::threeQuarter().toRadians()));
    EXPECT_EQ(move_primitive->move().dribbler_speed_rpm(), INDEFINITE_DRIBBLER_SPEED);
    ASSERT_TRUE(move_primitive->move().has_auto_chip_or_kick());
    EXPECT_EQ(move_primitive->move().auto_chip_or_kick().autokick_speed_m_per_s(), 3.5);
    EXPECT_EQ(move_primitive->move().max_speed_m_per_s(),
              STOP_COMMAND_ROBOT_MAX_SPEED_METERS_PER_SECOND);
}

TEST(PrimitiveFactoryTest, test_create_spinning_move_primitive)
{
    auto spinning_move_primitive = createSpinningMovePrimitive(
        Point(1, -8), -6.0, AngularVelocity::full(), DribblerMode::OFF);

    ASSERT_TRUE(spinning_move_primitive->has_spinning_move());
    EXPECT_EQ(spinning_move_primitive->spinning_move().destination().x_meters(), 1);
    EXPECT_EQ(spinning_move_primitive->spinning_move().destination().y_meters(), -8);
    EXPECT_EQ(spinning_move_primitive->spinning_move().final_speed_m_per_s(), -6.0);
    EXPECT_EQ(
        spinning_move_primitive->spinning_move().angular_velocity().radians_per_second(),
        static_cast<float>(AngularVelocity::full().toRadians()));
    EXPECT_EQ(spinning_move_primitive->spinning_move().dribbler_speed_rpm(), 0.0);
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
