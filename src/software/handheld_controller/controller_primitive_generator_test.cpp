#include "software/handheld_controller/controller_primitive_generator.h"

#include <gtest/gtest.h>

#include "shared/constants.h"

TEST(ControllerPrimitiveGeneratorTest, test_create_direct_velocity)
{
    auto direct_velocity_primitive =
        ControllerPrimitiveGenerator::createDirectVelocityPrimitive(
            Vector(2, -4), AngularVelocity::fromRadians(0.5), 200);

    ASSERT_TRUE(direct_velocity_primitive->has_direct_control());
    EXPECT_EQ(direct_velocity_primitive->direct_control()
                  .direct_velocity_control()
                  .velocity()
                  .x_component_meters(),
              2);
    EXPECT_EQ(direct_velocity_primitive->direct_control()
                  .direct_velocity_control()
                  .velocity()
                  .y_component_meters(),
              -4);
    EXPECT_EQ(direct_velocity_primitive->direct_control()
                  .direct_velocity_control()
                  .angular_velocity()
                  .radians_per_second(),
              0.5);
    EXPECT_EQ(direct_velocity_primitive->direct_control().dribbler_speed_rpm(), 200);
}
