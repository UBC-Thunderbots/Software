#pragma once

#include "shared/proto/primitive.pb.h"
#include "software/geom/angle.h"
#include "software/geom/angular_velocity.h"
#include "software/geom/point.h"
#include "software/util/make_enum/make_enum.h"

MAKE_ENUM(DribblerMode, OFF, INDEFINITE, MAX_FORCE);

/**
 * Create a Chip Primitive Message
 *
 * @param chip_origin The location where the chip will be taken
 * @param chip_direction The orientation the Robot will chip at
 * @param chip_distance_meters The distance between the starting location
 * of the chip and the location of the first bounce
 *
 * @return Pointer to Chip Primitive Message
 */
std::unique_ptr<TbotsProto::Primitive> createChipPrimitive(const Point &chip_origin,
                                                           const Angle &chip_direction,
                                                           double chip_distance_meters);

/**
 * Create a Kick Primitive Message
 *
 * @param kick_origin The location where the kick will be taken
 * @param kick_direction The orientation the Robot will kick at
 * @param kick_speed_meters_per_second The speed of how fast the Robot
 * will kick the ball in meters per second
 *
 * @return Pointer to Kick Primitive Message
 */
std::unique_ptr<TbotsProto::Primitive> createKickPrimitive(
    const Point &kick_origin, const Angle &kick_direction,
    double kick_speed_meters_per_second);

/**
 * Create a Move Primitive Message
 *
 * @param dest The final destination of the movement
 * @param final_speed_meters_per_second The speed at final destination
 * @param final_angle The final orientation the robot should have at the end
 * of the movement
 * @param dribbler_mode The dribbler mode
 *
 * @return Pointer to Move Primitive Message
 */
std::unique_ptr<TbotsProto::Primitive> createMovePrimitive(
    const Point &dest, double final_speed_meters_per_second, const Angle &final_angle,
    DribblerMode dribbler_mode);

/**
 * Create a Spinning Move Primitive Message
 *
 * @param dest The final destination of the movement
 * @param final_speed_meters_per_second The speed at final destination
 * @param angular_velocity The angular velocity of the robot
 * of the movement
 * @param dribbler_mode The dribbler mode
 *
 * @return Pointer to Spinning Move Primitive Message
 */
std::unique_ptr<TbotsProto::Primitive> createSpinningMovePrimitive(
    const Point &dest, double final_speed_meters_per_second,
    const AngularVelocity &angular_velocity, DribblerMode dribbler_mode);

/**
 * Create an Autochip Move Primitive Message
 *
 * @param dest The final destination of the movement
 * @param final_speed_meters_per_second The speed at final destination
 * @param final_angle The final orientation the robot should have at the end
 * of the movement
 * @param dribbler_mode The dribbler mode
 * @param chip_distance_meters The distance between the starting location
 * of the chip and the location of the first bounce
 *
 * @return Pointer to Autochip Move Primitive Message
 */
std::unique_ptr<TbotsProto::Primitive> createAutochipMovePrimitive(
    const Point &dest, double final_speed_meters_per_second, const Angle &final_angle,
    DribblerMode dribbler_mode, double chip_distance_meters);

/**
 * Create an Autokick Move Primitive Message
 *
 * @param dest The final destination of the movement
 * @param final_speed_meters_per_second The speed at final destination
 * @param final_angle The final orientation the robot should have at the end
 * of the movement
 * @param dribbler_mode The dribbler mode
 * @param kick_speed_meters_per_second The speed of how fast the Robot
 * will kick the ball in meters per second
 *
 * @return Pointer to Autokick Move Primitive Message
 */
std::unique_ptr<TbotsProto::Primitive> createAutokickMovePrimitive(
    const Point &dest, double final_speed_meters_per_second, const Angle &final_angle,
    DribblerMode dribbler_mode, double kick_speed_meters_per_second);

/**
 * Create a Stop Move Primitive Message
 *
 * @param stop_type Indicate to brake or coast to a stop
 *
 * @return Pointer to Stop Primitive Message
 */
std::unique_ptr<TbotsProto::Primitive> createStopPrimitive(bool coast);

/**
 * Convert dribbler mode to dribbler speed
 *
 * @param dribbler_mode The DribblerMode
 *
 * @return the dribbler speed in RPM
 */
double convertDribblerModeToDribblerSpeed(DribblerMode dribbler_mode);
