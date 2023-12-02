#pragma once

#include <optional>

#include "proto/primitive.pb.h"
#include "proto/primitive/primitive_types.h"
#include "shared/constants.h"
#include "shared/robot_constants.h"
#include "software/geom/angle.h"
#include "software/geom/angular_velocity.h"
#include "software/geom/point.h"
#include "software/util/make_enum/make_enum.h"
#include "software/world/robot.h"

/**
 * Create a Stop Primitive proto Message
 *
 * @return Pointer to Stop Primitive proto Message
 */
std::unique_ptr<TbotsProto::Primitive> createStopPrimitiveProto();

/**
 * Creates a new DirectControl Primitive AI could output this primitive to control the
 * linear velocity, angular velocity, and dribbler speed of a specific robot
 *
 * @param velocity x/y velocity vector
 * @param angular_velocity The angular velocity
 * @param dribbler_rpm The dribbler speed in rpm
 *
 * @return Pointer to the DirectControl Primitive
 */
std::unique_ptr<TbotsProto::Primitive> createDirectControlPrimitive(
    const Vector &velocity, AngularVelocity angular_velocity, double dribbler_rpm,
    const TbotsProto::AutoChipOrKick &auto_chip_or_kick);
