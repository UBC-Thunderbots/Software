#pragma once

#include "shared/constants.h"
#include "shared/proto/primitive.pb.h"
#include "software/geom/angle.h"
#include "software/geom/angular_velocity.h"
#include "software/geom/point.h"
#include "software/util/make_enum/make_enum.h"

/**
 * OFF - dribbler is off
 * INDEFINITE - the dribbler can be run at this speed indefinitely
 * MAX_FORCE - the dribbler applies maximum force to the ball
 */
MAKE_ENUM(DribblerMode, OFF, INDEFINITE, MAX_FORCE);

MAKE_ENUM(AutoChipOrKickMode, AUTOKICK, AUTOCHIP, OFF);

struct AutoChipOrKick
{
    AutoChipOrKickMode auto_chip_kick_mode;
    union
    {
        double autokick_speed_m_per_s;
        double autochip_distance_m;
    };
};

/**
 * PHYSICAL_LIMIT maximum speed allowed by the physical limits of the robot
 * STOP_COMMAND maximum speed allowed when responding to a stop command
 */
MAKE_ENUM(MaxAllowedSpeedMode, PHYSICAL_LIMIT, STOP_COMMAND);

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
 *
 * @return Pointer to Move Primitive Message
 */
std::unique_ptr<TbotsProto::Primitive> createMovePrimitive(
    const Point &dest, double final_speed_m_per_s, const Angle &final_angle,
    DribblerMode dribbler_mode, AutoChipOrKick auto_chip_or_kick,
    MaxAllowedSpeedMode max_allowed_speed_mode);

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

/**
 * Convert max allowed speed mode to max allowed speed
 *
 * @param max_allowed_speed_mode The MaxAllowedSpeedMode
 *
 * @return the max allowed speed in m/s
 */
double convertMaxAllowedSpeedModeToMaxAllowedSpeed(
    MaxAllowedSpeedMode max_allowed_speed_mode);
