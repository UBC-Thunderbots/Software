#pragma once

#include "shared/constants.h"
#include "software/ai/passing/pass.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/intersects.h"
#include "software/geom/circle.h"
#include "software/geom/segment.h"
#include "software/world/world.h"

/**
 * Finds all direct passes to open robots
 *
 * @param robot The robot with possession of the ball
 * @param friendly_team The same team of the robot with the possession of the ball
 * @param enemy_team The opposing team of the robot with the possession of the ball
 *
 * @return A vector containing all direct passes to open robots on the friendly_team
 */
std::vector<Pass> findDirectPasses(const Robot& robot, const Team& friendly_team,
                                   const Team& enemy_team);

/**
 * Finds all indirect passes to open robots
 *
 * @param robot The robot with possession of the ball
 * @param friendly_team The same team of the robot with the possession of the ball
 * @param enemy_team The opposing team of the robot with the possession of the ball
 *
 * @return A vector containing all indirect passes to open robots on the friendly_team
 */
std::vector<Pass> findIndirectPasses(const Robot& robot, const Team& friendly_team,
                                     const Team& enemy_team);
