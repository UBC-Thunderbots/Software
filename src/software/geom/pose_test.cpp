#include "software/geom/angle.h"
#include "software/geom/point.h"
#include "software/geom/pose.h"

#include <gtest/gtest.h>

TEST(PoseTest, DefaultConstructor)
{
    Pose pose;
    Point point = pose.point();
    Angle orientation = pose.orientation();

    EXPECT_EQ(Point(0, 0), point);
    EXPECT_EQ(Angle::zero(), orientation);
}

TEST(PoseTest, ParameterizedConstructorWithPointAndAngle)
{
    Point input_point(1.0, 2.0);
    Angle input_orientation = Angle::fromRadians(3.14);

    Pose pose(input_point, input_orientation);
    Point output_point = pose.point();
    Angle output_orientation = pose.orientation();

    EXPECT_EQ(input_point, output_point);
    EXPECT_EQ(input_orientation, output_orientation);
}

TEST(PoseTest, ParameterizedConstructorWithSeparateValues) {
    double x = 2.5;
    double y = -1.0;
    Angle orientation = Angle::fromRadians(0.5);

    Pose pose(x, y, orientation);
    Point output_point = pose.point();
    Angle output_orientation = pose.orientation();

    EXPECT_EQ(Point(x, y), output_point);
    EXPECT_EQ(orientation, output_orientation);
}
