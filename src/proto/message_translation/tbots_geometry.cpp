#include "proto/message_translation/tbots_geometry.h"

std::unique_ptr<TbotsProto::Point> createPointProto(const Point& point)
{
    auto point_msg = std::make_unique<TbotsProto::Point>();
    point_msg->set_x_meters(static_cast<float>(point.x()));
    point_msg->set_y_meters(static_cast<float>(point.y()));
    return point_msg;
}

std::unique_ptr<TbotsProto::Angle> createAngleProto(const Angle& angle)
{
    auto angle_msg = std::make_unique<TbotsProto::Angle>();
    angle_msg->set_radians(static_cast<float>(angle.toRadians()));
    return angle_msg;
}

std::unique_ptr<TbotsProto::AngularVelocity> createAngularVelocityProto(
    const AngularVelocity& angular_velocity)
{
    auto anglular_velocity_msg = std::make_unique<TbotsProto::AngularVelocity>();
    anglular_velocity_msg->set_radians_per_second(
        static_cast<float>(angular_velocity.toRadians()));
    return anglular_velocity_msg;
}

std::unique_ptr<TbotsProto::Vector> createVectorProto(const Vector& vector)
{
    auto vector_msg = std::make_unique<TbotsProto::Vector>();
    vector_msg->set_x_component_meters(static_cast<float>(vector.x()));
    vector_msg->set_y_component_meters(static_cast<float>(vector.y()));
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
