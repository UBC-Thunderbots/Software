#pragma once

#include "shared/proto/primitive.pb.h"
#include "software/geom/angle.h"
#include "software/geom/angular_velocity.h"
#include "software/geom/point.h"
#include "software/primitive/primitive.h"

/**
 * Enum for indicating the stop type for the Stop Primitive
 */
enum StopType
{
    BRAKE,
    COAST
};

/**
 * Create a Chip Primitive
 *
 * @param chip_origin The location where the chip will be taken
 * @param chip_direction The orientation the Robot will chip at
 * @param chip_distance_meters The distance between the starting location
 * of the chip and the location of the first bounce
 *
 * @return Pointer to Chip Primitive
 */
std::unique_ptr<ChipPrimitiveMsg> createChipPrimitiveMsg(const Point &chip_origin,
                                                         const Angle &chip_direction,
                                                         float chip_distance_meters);

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
std::unique_ptr<KickPrimitiveMsg> createKickPrimitiveMsg(
    const Point &kick_origin, const Angle &kick_direction,
    float kick_speed_meters_per_second);

/**
 * Create a Move Primitive Message
 *
 * @param dest The final destination of the movement
 * @param final_speed_meters_per_second The speed at final destination
 * @param slow Whether or not to move at a slower speed (1m/s)
 * @param final_angle The final orientation the robot should have at the end
 * of the movement
 * @param dribbler_speed_rpm The speed of how fast the dribbler runs
 *
 * @return Pointer to Move Primitive Message
 */
std::unique_ptr<MovePrimitiveMsg> createMovePrimitiveMsg(
    const Point &dest, float final_speed_meters_per_second, bool slow,
    const Angle &final_angle, float dribbler_speed_rpm);

/**
 * Create a Spinning Move Primitive Message
 *
 * @param dest The final destination of the movement
 * @param final_speed_meters_per_second The speed at final destination
 * @param slow Whether or not to move at a slower speed (1m/s)
 * @param angular_velocity The angular velocity of the robot
 * of the movement
 * @param dribbler_speed_rpm The speed of how fast the dribbler runs
 *
 * @return Pointer to Spinning Move Primitive Message
 */
std::unique_ptr<SpinningMovePrimitiveMsg> createSpinningMovePrimitiveMsg(
    const Point &dest, float final_speed_meters_per_second, bool slow,
    const AngularVelocity &angular_velocity, float dribbler_speed_rpm);

/**
 * Create an Autochip Move Primitive Message
 *
 * @param dest The final destination of the movement
 * @param final_speed_meters_per_second The speed at final destination
 * @param slow Whether or not to move at a slower speed (1m/s)
 * @param final_angle The final orientation the robot should have at the end
 * of the movement
 * @param dribbler_speed_rpm The speed of how fast the dribbler runs
 * @param chip_distance_meters The distance between the starting location
 * of the chip and the location of the first bounce
 *
 * @return Pointer to Autochip Move Primitive Message
 */
std::unique_ptr<AutochipMovePrimitiveMsg> createAutochipMovePrimitiveMsg(
    const Point &dest, float final_speed_meters_per_second, bool slow,
    const Angle &final_angle, float dribbler_speed_rpm, float chip_distance_meters);

/**
 * Create an Autokick Move Primitive Message
 *
 * @param dest The final destination of the movement
 * @param final_speed_meters_per_second The speed at final destination
 * @param slow Whether or not to move at a slower speed (1m/s)
 * @param final_angle The final orientation the robot should have at the end
 * of the movement
 * @param dribbler_speed_rpm The speed of how fast the dribbler runs
 * @param kick_speed_meters_per_second The speed of how fast the Robot
 * will kick the ball in meters per second
 *
 * @return Pointer to Autokick Move Primitive Message
 */
std::unique_ptr<AutokickMovePrimitiveMsg> createAutokickMovePrimitiveMsg(
    const Point &dest, float final_speed_meters_per_second, bool slow,
    const Angle &final_angle, float dribbler_speed_rpm,
    float kick_speed_meters_per_second);

/**
 * Create a Stop Move Primitive Message
 *
 * @param stop_type Indicate to brake or coast to a stop
 *
 * @return Pointer to Stop Primitive Message
 */
std::unique_ptr<StopPrimitiveMsg> createStopPrimitiveMsg(StopType stop_type);
