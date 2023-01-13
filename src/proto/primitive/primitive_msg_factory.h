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

/**
 * Create a Move Primitive Message
 *
 * @param dest The final destination of the movement
 * @param final_speed_m_per_s The speed at final destination
 * @param final_angle The final orientation the robot should have at the end
 * of the movement
 * @param dribbler_mode The dribbler mode
 * @param auto_chip_or_kick The command to autochip or autokick
 * @param max_allowed_speed_mode The mode of maximum speed allowed
 * @param target_spin_rev_per_s The target spin while moving in revolutions per second
 * @param robot_constants The robot constants
 * @param cost_override optionally override the cost of the move primitive, defaults to
 * the path length
 *
 * @return Pointer to Move Primitive Message
 */
std::unique_ptr<TbotsProto::Primitive> createMovePrimitive(
    const TbotsProto::MotionControl& motion_control, const Angle& final_angle,
    double final_speed, const TbotsProto::DribblerMode& dribbler_mode,
    const TbotsProto::BallCollisionType& ball_collision_type,
    const AutoChipOrKick& auto_chip_or_kick,
    const TbotsProto::MaxAllowedSpeedMode& max_allowed_speed_mode,
    double target_spin_rev_per_s, const RobotConstants_t& robot_constants,
    std::optional<double> cost_override = std::nullopt);

/**
 * Create a Stop Move Primitive Message
 *
 * @return Pointer to Stop Primitive Message
 */
std::unique_ptr<TbotsProto::Primitive> createStopPrimitive();

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
    const Vector& velocity, AngularVelocity angular_velocity, double dribbler_rpm,
    const TbotsProto::AutoChipOrKick& auto_chip_or_kick);


/**
 * Convert dribbler mode to dribbler speed
 *
 * @param dribbler_mode The DribblerMode
 * @param robot_constants The robot constants
 *
 * @return the dribbler speed in RPM
 */
double convertDribblerModeToDribblerSpeed(TbotsProto::DribblerMode dribbler_mode,
                                          RobotConstants_t robot_constants);

/**
 * Convert max allowed speed mode to max allowed speed
 *
 * @param max_allowed_speed_mode The MaxAllowedSpeedMode
 * @param robot_constants The robot constants
 *
 * @return the max allowed speed in m/s
 */
double convertMaxAllowedSpeedModeToMaxAllowedSpeed(
    TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode,
    RobotConstants_t robot_constants);
