#include "proto/message_translation/tbots_geometry.h"

#include <google/protobuf/util/message_differencer.h>
#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

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

TEST(TbotsProtobufTest, polygon_msg_test)
{
    auto polygon = Polygon({Point(4.20, 4.20), Point(1.0, 1.1), Point(-1.0, -153.52)});
    auto polygon_msg = createPolygonProto(polygon);

    EXPECT_EQ(polygon.getPoints().size(), polygon_msg->points_size());

    EXPECT_EQ(polygon.getPoints()[0].x(), polygon_msg->points(0).x_meters());
    EXPECT_EQ(polygon.getPoints()[0].y(), polygon_msg->points(0).y_meters());

    EXPECT_EQ(polygon.getPoints()[1].x(), polygon_msg->points(1).x_meters());
    EXPECT_EQ(polygon.getPoints()[1].y(), polygon_msg->points(1).y_meters());

    EXPECT_EQ(polygon.getPoints()[2].x(), polygon_msg->points(2).x_meters());
    EXPECT_EQ(polygon.getPoints()[2].y(), polygon_msg->points(2).y_meters());
}

TEST(TbotsProtobufTest, circle_msg_test)
{
    auto circle_1     = Circle(Point(-1, 1), 4);
    auto circle_msg_1 = createCircleProto(circle_1);
    auto circle_2     = createCircle(*circle_msg_1);
    auto circle_msg_2 = createCircleProto(circle_2);

    EXPECT_TRUE(
        google::protobuf::util::MessageDifferencer::Equals(*circle_msg_1, *circle_msg_2));
}

TEST(TbotsProtobufTest, velocity_obstacle_msg_test)
{
    Vector apex  = Vector(3, 3);
    Vector side1 = Vector(-1, -2);
    Vector side2 = Vector(1, 2);

    VelocityObstacle velocity_obstacle = VelocityObstacle(apex, side1, side2);
    auto velocity_obstacle_msg_1       = createVelocityObstacleProto(velocity_obstacle);
    auto velocity_obstacle_2           = createVelocityObstacle(*velocity_obstacle_msg_1);
    auto velocity_obstacle_msg_2       = createVelocityObstacleProto(velocity_obstacle_2);

    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        *velocity_obstacle_msg_1, *velocity_obstacle_msg_2));
}

TEST(TbotsProtobufTest, stadium_msg_test)
{
    auto stadium_1 = Stadium(Segment(Point(1, 2), Point(0,-2)), 2);
    auto stadium_msg_1 = createStadiumProto(stadium_1);
    auto stadium_2 = createStadium(*stadium_msg_1);
    auto stadium_msg_2 = createStadiumProto(stadium_2);

    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(*stadium_msg_1, *stadium_msg_2));
}
