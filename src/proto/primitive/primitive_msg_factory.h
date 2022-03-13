#pragma once

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
 *
 * @return Pointer to Move Primitive Message
 */
std::unique_ptr<TbotsProto::Primitive> createMovePrimitive(
    const Point &dest, double final_speed_m_per_s, const Angle &final_angle,
    TbotsProto::DribblerMode dribbler_mode, AutoChipOrKick auto_chip_or_kick,
    TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode, double target_spin_rev_per_s,
    RobotConstants_t robot_constants);

std::unique_ptr<TbotsProto::Primitive> createMovePrimitive(
    const Point& destination, const Angle& final_angle, double final_speed,
    const TbotsProto::DribblerMode& dribbler_mode, const BallCollisionType& ball_collision_type,
    const AutoChipOrKick& auto_chip_or_kick,
    const TbotsProto::MaxAllowedSpeedMode& max_allowed_speed_mode, double target_spin_rev_per_s,
    // TODO: delete default argument
    const RobotConstants_t& robot_constants, double cost = 1.0);

std::unique_ptr<TbotsProto::Primitive> createChipPrimitive(
    const Point& chip_origin, const Angle& chip_direction, double chip_distance_meters,
    // TODO: delete default argument
    RobotConstants_t robot_constants, double cost = 1.0);

std::unique_ptr<TbotsProto::Primitive> createKickPrimitive(
    const Point& kick_origin, const Angle& kick_direction,
    double kick_speed_meters_per_second,
    // TODO: delete default argument
    RobotConstants_t robot_constants, double cost = 1.0);

/**
 * Create a Stop Move Primitive Message
 *
 * @param stop_type Indicate to brake or coast to a stop
 *
 * @return Pointer to Stop Primitive Message
 */
std::unique_ptr<TbotsProto::Primitive> createStopPrimitive(bool coast);

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
    const Vector& velocity, AngularVelocity angular_velocity, double dribbler_rpm);



/**
 * Create a Estop Primitive Message
 *
 * @return Pointer to Stop Primitive Message
 */
std::unique_ptr<TbotsProto::Primitive> createEstopPrimitive();

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
