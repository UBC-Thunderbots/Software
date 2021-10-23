#include "proto/message_translation/tbots_geometry.h"

#include <gtest/gtest.h>

TEST(TbotsProtobufTest, point_msg_test)
{
    auto point     = Point(4.20, 4.20);
    auto point_msg = createPointProto(point);

    EXPECT_NEAR(point_msg->x_meters(), point.x(), 1e-6);
    EXPECT_NEAR(point_msg->y_meters(), point.y(), 1e-6);
}

TEST(TbotsProtobufTest, angular_velocity_msg_test)
{
    auto angular_velocity     = Angle::fromRadians(4.20);
    auto angular_velocity_msg = createAngularVelocityProto(angular_velocity);

    EXPECT_NEAR(angular_velocity_msg->radians_per_second(), angular_velocity.toRadians(),
                1e-6);
}

TEST(TbotsProtobufTest, angle_msg_test)
{
    auto angle     = Angle::fromRadians(4.20);
    auto angle_msg = createAngleProto(angle);

    EXPECT_NEAR(angle_msg->radians(), angle.toRadians(), 1e-6);
}

TEST(TbotsProtobufTest, vector_msg_test)
{
    auto vector     = Vector(4.20, 4.20);
    auto vector_msg = createVectorProto(vector);

    EXPECT_NEAR(vector_msg->x_component_meters(), vector.x(), 1e-6);
    EXPECT_NEAR(vector_msg->y_component_meters(), vector.y(), 1e-6);
}
