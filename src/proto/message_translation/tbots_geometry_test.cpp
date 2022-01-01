#include "proto/message_translation/tbots_geometry.h"

#include <gtest/gtest.h>

TEST(TbotsProtobufTest, point_msg_test)
{
    auto point     = Point(4.20, 4.20);
    auto point_msg = createPointProto(point);

    EXPECT_EQ(point_msg->x_meters(), point.x());
    EXPECT_EQ(point_msg->y_meters(), point.y());
}

TEST(TbotsProtobufTest, angular_velocity_msg_test)
{
    auto angular_velocity     = Angle::fromRadians(4.20);
    auto angular_velocity_msg = createAngularVelocityProto(angular_velocity);

    EXPECT_EQ(angular_velocity_msg->radians_per_second(), angular_velocity.toRadians());
}

TEST(TbotsProtobufTest, angle_msg_test)
{
    auto angle     = Angle::fromRadians(4.20);
    auto angle_msg = createAngleProto(angle);

    EXPECT_EQ(angle_msg->radians(), angle.toRadians());
}

TEST(TbotsProtobufTest, vector_msg_test)
{
    auto vector     = Vector(4.20, 4.20);
    auto vector_msg = createVectorProto(vector);

    EXPECT_EQ(vector_msg->x_component_meters(), vector.x());
    EXPECT_EQ(vector_msg->y_component_meters(), vector.y());
}
