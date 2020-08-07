#include "software/proto/message_translation/tbots_geometry.h"

std::unique_ptr<PointMsg> createPointMsg(const Point& point)
{
    auto point_msg = std::make_unique<PointMsg>();
    point_msg->set_x_meters(static_cast<float>(point.x()));
    point_msg->set_y_meters(static_cast<float>(point.y()));
    return std::move(point_msg);
}

std::unique_ptr<AngleMsg> createAngleMsg(const Angle& angle)
{
    auto angle_msg = std::make_unique<AngleMsg>();
    angle_msg->set_radians(static_cast<float>(angle.toRadians()));
    return std::move(angle_msg);
}

std::unique_ptr<AngularVelocityMsg> createAngularVelocityMsg(
    const AngularVelocity& angular_velocity)
{
    auto anglular_velocity_msg = std::make_unique<AngularVelocityMsg>();
    anglular_velocity_msg->set_radians_per_second(
        static_cast<float>(angular_velocity.toRadians()));
    return std::move(anglular_velocity_msg);
}

std::unique_ptr<VectorMsg> createVectorMsg(const Vector& vector)
{
    auto vector_msg = std::make_unique<VectorMsg>();
    vector_msg->set_x_component_meters(static_cast<float>(vector.x()));
    vector_msg->set_y_component_meters(static_cast<float>(vector.y()));
    return std::move(vector_msg);
}
