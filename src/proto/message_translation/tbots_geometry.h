#pragma once

#include "proto/geometry.pb.h"
#include "software/geom/angle.h"
#include "software/geom/angular_velocity.h"
#include "software/geom/circle.h"
#include "software/geom/point.h"
#include "software/geom/polygon.h"

/**
 * Internal geometry types to protobuf msg conversions
 *
 * @param The geom type to convert to proto
 *
 * @return The unique_ptr to the converted Geom proto
 */
std::unique_ptr<TbotsProto::Point> createPointProto(const Point& point);
std::unique_ptr<TbotsProto::Angle> createAngleProto(const Angle& angle);
std::unique_ptr<TbotsProto::AngularVelocity> createAngularVelocityProto(
    const AngularVelocity& angular_velocity);
std::unique_ptr<TbotsProto::Vector> createVectorProto(const Vector& vector);
std::unique_ptr<TbotsProto::Polygon> createPolygonProto(const Polygon& polygon);
<<<<<<< HEAD
=======
std::unique_ptr<TbotsProto::Circle> createCircleProto(const Circle& polygon);
>>>>>>> 0993cd662b74b714202ffaf0b5bedf2df4e1a088

/**
 * Protobuf msg types to internal geometry types conversions
 *
 * @param The proto to convert to geom type
 *
 * @return The equivalent geom object
 */
Point createPoint(const TbotsProto::Point& point);
Angle createAngle(const TbotsProto::Angle& angle);
AngularVelocity createAngularVelocity(
    const TbotsProto::AngularVelocity& angular_velocity);
Vector createVector(const TbotsProto::Vector& vector);
Polygon createPolygon(const TbotsProto::Polygon& polygon);
<<<<<<< HEAD
=======
Circle createCircle(const TbotsProto::Circle& circle);
>>>>>>> 0993cd662b74b714202ffaf0b5bedf2df4e1a088
