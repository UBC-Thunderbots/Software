#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/passing/pass.h"
#include "software/world/world.h"


/**
 * Find a point relatively close to the ball, that we can dribble it toward to maximize
 * the quality of a pass to the receiver point in the best pass so far, while also
 * trying to get far away from enemy robots.
 *
 * @param world the world
 * @param best_pass_so_far the best pass so far
 * @param passing_config the passing config
 * @return a Point with the qualities described above.
 */
Point findKeepAwayTargetPoint(const World& world, const Pass& best_pass_so_far,
                              const TbotsProto::PassingConfig& passing_config);

/**
 * Returns true if the given robot is being threatened by enemy robots close
 * to the robot. If true, the robot should move to get far away from
 * the enemy robots.
 *
 * @param robot the robot to check
 * @param enemy_team the enemy team
 * @param about_to_steal_radius radius of a circle in meters around the robot
 * inside which we consider an enemy is about to steal the ball
 *
 * @return true if the robot is being threatened by the enemy team
 */
bool shouldKeepAway(const Robot& robot, const Team& enemy_team,
                    double about_to_steal_radius);
