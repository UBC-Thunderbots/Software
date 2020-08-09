#pragma once

#include "shared/proto/geometry.pb.h"
#include "software/geom/angle.h"
#include "software/geom/angular_velocity.h"
#include "software/geom/point.h"

/**
 * Internal geometry types to protobuf msg conversions
 *
 * @param The geom type to convert to proto
 *
 * @return The unique_ptr to the converted Geom proto
 */
std::unique_ptr<TbotsProto::Point> createPoint(const Point& point);
std::unique_ptr<TbotsProto::Angle> createAngle(const Angle& angle);
std::unique_ptr<TbotsProto::AngularVelocity> createAngularVelocity(
    const AngularVelocity& angular_velocity);
std::unique_ptr<TbotsProto::Vector> createVector(const Vector& vector);
