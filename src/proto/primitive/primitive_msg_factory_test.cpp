#include "proto/primitive/primitive_msg_factory.h"

#include <gtest/gtest.h>

#include "shared/constants.h"
#include "shared/robot_constants_2021.h"
#include "software/test_util/test_util.h"

class PrimitiveFactoryTest : public testing::Test
{
   protected:
    RobotConstants_t robot_constants = create2021RobotConstants();
};

TEST_F(PrimitiveFactoryTest, test_auto_chip_or_kick_equality)
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

TEST_F(PrimitiveFactoryTest, test_create_move_primitive)
{
    auto move_primitive = createMovePrimitive(
        TestUtil::createMotionControl(Point(-5, 1)), Angle::threeQuarter(), 3.0,
        TbotsProto::DribblerMode::INDEFINITE, TbotsProto::BallCollisionType::AVOID,
        {AutoChipOrKickMode::OFF, 0}, TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        5.0, robot_constants);

    ASSERT_TRUE(move_primitive->has_move());
    auto destination = move_primitive->move().motion_control().path().points().at(0);
    EXPECT_EQ(destination.x_meters(), -5);
    EXPECT_EQ(destination.y_meters(), 1);
    EXPECT_EQ(move_primitive->move().final_speed_m_per_s(), 3.0);
    EXPECT_EQ(move_primitive->move().final_angle().radians(),
              Angle::threeQuarter().toRadians());
    EXPECT_EQ(move_primitive->move().dribbler_speed_rpm(),
              robot_constants.indefinite_dribbler_speed_rpm);
    EXPECT_FALSE(move_primitive->move().has_auto_chip_or_kick());
    EXPECT_EQ(move_primitive->move().max_speed_m_per_s(),
              robot_constants.robot_max_speed_m_per_s);
    EXPECT_EQ(move_primitive->move().target_spin_rev_per_s(), 5);
}

TEST_F(PrimitiveFactoryTest, test_create_move_primitive_with_autochip)
{
    auto move_primitive = createMovePrimitive(
        TestUtil::createMotionControl(Point(-5, 1)), Angle::threeQuarter(), 3.0,
        TbotsProto::DribblerMode::INDEFINITE, TbotsProto::BallCollisionType::AVOID,
        {AutoChipOrKickMode::AUTOCHIP, 2.5},
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0, robot_constants);


    ASSERT_TRUE(move_primitive->has_move());
    auto destination = move_primitive->move().motion_control().path().points().at(0);
    EXPECT_EQ(destination.x_meters(), -5);
    EXPECT_EQ(destination.y_meters(), 1);
    EXPECT_EQ(move_primitive->move().final_speed_m_per_s(), 3.0);
    EXPECT_EQ(move_primitive->move().final_angle().radians(),
              Angle::threeQuarter().toRadians());
    EXPECT_EQ(move_primitive->move().dribbler_speed_rpm(),
              robot_constants.indefinite_dribbler_speed_rpm);
    ASSERT_TRUE(move_primitive->move().has_auto_chip_or_kick());
    EXPECT_EQ(move_primitive->move().auto_chip_or_kick().autochip_distance_meters(), 2.5);
    EXPECT_EQ(move_primitive->move().max_speed_m_per_s(),
              robot_constants.robot_max_speed_m_per_s);
    EXPECT_EQ(move_primitive->move().target_spin_rev_per_s(), 0.0f);
}

TEST_F(PrimitiveFactoryTest, test_create_move_primitive_with_autokick)
{
    auto move_primitive = createMovePrimitive(
        TestUtil::createMotionControl(Point(-5, 1)), Angle::threeQuarter(), 3.0,
        TbotsProto::DribblerMode::INDEFINITE, TbotsProto::BallCollisionType::AVOID,
        {AutoChipOrKickMode::AUTOKICK, 3.5},
        TbotsProto::MaxAllowedSpeedMode::STOP_COMMAND, 0.0, robot_constants);

    ASSERT_TRUE(move_primitive->has_move());
    auto destination = move_primitive->move().motion_control().path().points().at(0);
    EXPECT_EQ(destination.x_meters(), -5);
    EXPECT_EQ(destination.y_meters(), 1);
    EXPECT_EQ(move_primitive->move().final_speed_m_per_s(), 3.0);
    EXPECT_EQ(move_primitive->move().final_angle().radians(),
              Angle::threeQuarter().toRadians());
    EXPECT_EQ(move_primitive->move().dribbler_speed_rpm(),
              robot_constants.indefinite_dribbler_speed_rpm);
    ASSERT_TRUE(move_primitive->move().has_auto_chip_or_kick());
    EXPECT_EQ(move_primitive->move().auto_chip_or_kick().autokick_speed_m_per_s(), 3.5);
    EXPECT_EQ(move_primitive->move().max_speed_m_per_s(),
              STOP_COMMAND_ROBOT_MAX_SPEED_METERS_PER_SECOND);
    EXPECT_EQ(move_primitive->move().target_spin_rev_per_s(), 0.0f);
}

TEST_F(PrimitiveFactoryTest, test_create_direct_velocity)
{
    auto direct_velocity_primitive =
        createDirectControlPrimitive(Vector(2, -4), AngularVelocity::fromRadians(0.5),
                                     200, TbotsProto::AutoChipOrKick());

    ASSERT_TRUE(direct_velocity_primitive->has_direct_control());
    EXPECT_EQ(direct_velocity_primitive->direct_control()
                  .motor_control()
                  .direct_velocity_control()
                  .velocity()
                  .x_component_meters(),
              2);
    EXPECT_EQ(direct_velocity_primitive->direct_control()
                  .motor_control()
                  .direct_velocity_control()
                  .velocity()
                  .y_component_meters(),
              -4);
    EXPECT_EQ(direct_velocity_primitive->direct_control()
                  .motor_control()
                  .direct_velocity_control()
                  .angular_velocity()
                  .radians_per_second(),
              0.5);
    EXPECT_EQ(
        direct_velocity_primitive->direct_control().motor_control().dribbler_speed_rpm(),
        200);
}



TEST_F(PrimitiveFactoryTest, test_create_stop_primitive_brake)
{
    auto stop_primitive = createStopPrimitive(false);

    ASSERT_TRUE(stop_primitive->has_stop());
    EXPECT_EQ(stop_primitive->stop().stop_type(), TbotsProto::StopPrimitive::BRAKE);
}

TEST_F(PrimitiveFactoryTest, test_create_stop_primitive_coast)
{
    auto stop_primitive = createStopPrimitive(true);

    ASSERT_TRUE(stop_primitive->has_stop());
    EXPECT_EQ(stop_primitive->stop().stop_type(), TbotsProto::StopPrimitive::COAST);
}

TEST_F(PrimitiveFactoryTest, test_create_estop_primitive)
{
    auto Estop_primitive = createEstopPrimitive();

    ASSERT_TRUE(Estop_primitive->has_estop());
}
