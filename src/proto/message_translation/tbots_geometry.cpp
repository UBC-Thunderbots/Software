#include "proto/message_translation/tbots_geometry.h"

std::unique_ptr<TbotsProto::Point> createPointProto(const Point& point)
{
    auto point_msg = std::make_unique<TbotsProto::Point>();
    point_msg->set_x_meters(point.x());
    point_msg->set_y_meters(point.y());
    return point_msg;
}

std::unique_ptr<TbotsProto::Angle> createAngleProto(const Angle& angle)
{
    auto angle_msg = std::make_unique<TbotsProto::Angle>();
    angle_msg->set_radians(angle.toRadians());
    return angle_msg;
}

std::unique_ptr<TbotsProto::AngularVelocity> createAngularVelocityProto(
    const AngularVelocity& angular_velocity)
{
    auto anglular_velocity_msg = std::make_unique<TbotsProto::AngularVelocity>();
    anglular_velocity_msg->set_radians_per_second(
        angular_velocity.toRadians());
    return anglular_velocity_msg;
}

std::unique_ptr<TbotsProto::Vector> createVectorProto(const Vector& vector)
{
    auto vector_msg = std::make_unique<TbotsProto::Vector>();
    vector_msg->set_x_component_meters(vector.x());
    vector_msg->set_y_component_meters(vector.y());
    return vector_msg;
}

std::unique_ptr<TbotsProto::Polygon> createPolygonProto(const Polygon& polygon)
{
    auto polygon_msg   = std::make_unique<TbotsProto::Polygon>();
    const auto& points = polygon.getPoints();

    std::for_each(points.begin(), points.end(), [&](const Point& point) {
        *(polygon_msg->add_points()) = *createPointProto(point);
    });

    return polygon_msg;
}

Point createPoint(const TbotsProto::Point& point)
{
    return Point(point.x_meters(), point.y_meters());
}

Angle createAngle(const TbotsProto::Angle& angle)
{
    return Angle::fromRadians(angle.radians());
}

AngularVelocity createAngularVelocity(const TbotsProto::AngularVelocity& angular_velocity)
{
    return AngularVelocity::fromRadians(angular_velocity.radians_per_second());
}

Vector createVector(const TbotsProto::Vector& vector)
{
    return Vector(vector.x_component_meters(), vector.y_component_meters());
}

Polygon createPolygon(const TbotsProto::Polygon& polygon)
{
    std::vector<Point> polygon_points;
    const auto& polygons_msg_points = polygon.points();
    for (const TbotsProto::Point& point : polygons_msg_points)
    {
        polygon_points.emplace_back(createPoint(point));
    }

    return Polygon(polygon_points);
}
