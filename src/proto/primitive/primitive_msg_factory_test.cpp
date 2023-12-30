#include "proto/primitive/primitive_msg_factory.h"

#include <gtest/gtest.h>

#include "shared/2021_robot_constants.h"
#include "shared/constants.h"
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
    auto stop_primitive = createStopPrimitiveProto();

    ASSERT_TRUE(stop_primitive->has_stop());
}
