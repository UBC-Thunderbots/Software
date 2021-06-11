#include "software/proto/primitive/primitive_msg_factory.h"

#include <gtest/gtest.h>

#include "shared/constants.h"

TEST(PrimitiveFactoryTest, test_auto_chip_or_kick_equality)
{
    AutoChipOrKick auto_chip_or_kick       = {AutoChipOrKickMode::AUTOCHIP, 3.2};
    AutoChipOrKick auto_chip_or_kick_other = {AutoChipOrKickMode::AUTOCHIP, 3.2};
    EXPECT_EQ(auto_chip_or_kick, auto_chip_or_kick_other);
    auto_chip_or_kick       = {AutoChipOrKickMode::OFF, 3.2};
    auto_chip_or_kick_other = {AutoChipOrKickMode::AUTOCHIP, 3.2};
    EXPECT_NE(auto_chip_or_kick, auto_chip_or_kick_other);
    auto_chip_or_kick       = {AutoChipOrKickMode::OFF, 3.2};
    auto_chip_or_kick_other = {AutoChipOrKickMode::OFF, 2};
    EXPECT_EQ(auto_chip_or_kick, auto_chip_or_kick_other);
}

TEST(PrimitiveFactoryTest, test_create_move_primitive)
{
    auto move_primitive = createMovePrimitive(
        Point(-5, 1), 3.0, Angle::threeQuarter(), DribblerMode::INDEFINITE,
        {AutoChipOrKickMode::OFF, 0}, MaxAllowedSpeedMode::PHYSICAL_LIMIT, 5.0);

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
    EXPECT_EQ(move_primitive->move().target_spin_rev_per_s(), 5);
}

TEST(PrimitiveFactoryTest, test_create_move_primitive_with_autochip)
{
    auto move_primitive = createMovePrimitive(
        Point(-5, 1), 3.0, Angle::threeQuarter(), DribblerMode::INDEFINITE,
        {AutoChipOrKickMode::AUTOCHIP, 2.5}, MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0);

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
    EXPECT_EQ(move_primitive->move().target_spin_rev_per_s(), 0.0f);
}

TEST(PrimitiveFactoryTest, test_create_move_primitive_with_autokick)
{
    auto move_primitive = createMovePrimitive(
        Point(-5, 1), 3.0, Angle::threeQuarter(), DribblerMode::INDEFINITE,
        {AutoChipOrKickMode::AUTOKICK, 3.5}, MaxAllowedSpeedMode::STOP_COMMAND, 0.0);

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
    EXPECT_EQ(move_primitive->move().target_spin_rev_per_s(), 0.0f);
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

TEST(PrimitiveFactoryTest, test_create_estop_primitive)
{
    auto Estop_primitive = createEstopPrimitive();

    ASSERT_TRUE(Estop_primitive->has_estop());
}
