#pragma once

#include "ai/world/team.h"
#include "ai/geom/point.h"
// Requires geom classes.
/**
 * This defines the interface for obstacle processing
 */



/**
 * Creates circles around the friendly team robots.
 * 
 * @param friendly_team The team of friendly robots.
 * @param avoid_dist Avoid distance around robots.
 */
std::vector<Geom::Circle> process_friendly(Team& friendly_team, double avoid_dist);

/**
 * Creates circles around the enemy team robots.
 * 
 * @param enemy_team The team of enemy robots.
 * @param avoid_dist Avoid distance around robots.
 */
std::vector<Geom::Circle> process_enemy(Team& enemy_team, double avoid_dist);

bool path_valid(std::vector<Point>);