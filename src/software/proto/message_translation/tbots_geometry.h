#pragma once

#include "shared/proto/geometry.pb.h"
#include "software/new_geom/angle.h"
#include "software/new_geom/angular_velocity.h"
#include "software/new_geom/point.h"

/**
 * Internal geometry types to protobuf msg conversions
 *
 * @param The geom type to convert to proto
 * @return The unique_ptr to the converted GeomMsg
 */
std::unique_ptr<PointMsg> createPointMsg(const Point& point);
std::unique_ptr<AngleMsg> createAngleMsg(const Angle& angle);
std::unique_ptr<AngularVelocityMsg> createAngularVelocityMsg(
    const AngularVelocity& angular_velocity);
std::unique_ptr<VectorMsg> createVectorMsg(const Vector& vector);
