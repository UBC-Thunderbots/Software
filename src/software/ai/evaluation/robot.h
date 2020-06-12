#pragma once

#include "software/new_geom/point.h"
#include "software/world/world.h"

/**
 * Independent Evaluation function to evaluate whether robot
 * orientation at position is within threshold
 */

/**
 * Determines whether or not the robot at position
 * is facing within threshold degrees of the specified target
 *
 * @param position current position of the robot
 * @param orientation current orientation of the robot
 * @param target destination coordinate of the robot
 * @param threshold the upper bound of angle for evaluation
 *
 * @return True if angle formed between orientation direction and the
 * direction from position to target is smaller than threshold angle,
 * false otherwise
 */
bool robotOrientationWithinAngleThresholdOfTarget(Point position, Angle orientation,
                                                  Point target, Angle threshold);

/**
 * Determines if a robot has possession of the ball. A robot is considered to have
 * possession if the ball is in a area close to its dribbler.
 *
 * @param  The ball that is wanted to be possessed
 * @param robot The Robot which wants to know if it has the ball.
 * @param timestamp The time at which we want to know if the robot had the ball
 * @return True if the ball is close to the front dribbler, False if the ball is not,
 *          and nullopt if we don't have information for the given timestamp. This
 * function is guaranteed to return non-nullopt if no timestamp is passed in.
 */
std::optional<bool> robotHasPossession(const Ball& ball, const Robot& robot,
                                       std::optional<Timestamp> timestamp = std::nullopt);

/**
 * Returns true if the robot is being passed to. A robot is considered to be being
 * passed to if the ball is heading toward it above a certain speed.
 * @param world The world
 * @param robot The robot
 * @param timestamp the timestamp of the world and robot states we want to consider
 * @return True if the robot is being passed to at the given time, False if the robot
 * is not being passed to, and nullopt if we don't have information for the given
 * timestamp. This function is guaranteed to return non-nullopt if no timestamp is
 * passed in.
 */
std::optional<bool> robotBeingPassedTo(const World& world, const Robot& robot,
                                       std::optional<Timestamp> timestamp = std::nullopt);
