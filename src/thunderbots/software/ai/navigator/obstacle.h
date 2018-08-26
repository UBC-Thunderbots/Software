#pragma once

#include "ai/world/team.h"
#include "ai/geom/point.h"

/**
 * This defines the interface for obstacle processing.
 */

constexpr double DEFAULT_AVOID_DIST = 0.15; // meters

typedef struct _RobotObstacle
{
    /**
     * Represents the physical footprint of the robot, with
     * an additional avoid distance.
     */
    Geom::Circle robot;

    /**
     * Segment extending from robot in direction of robot travel with length
     * proportional to current velocity. Used for collision avoidance.
     */
    Geom::Seg velocity;

} RobotObstacle;


typedef struct _FieldBoundary
{
    /**
     * Since all field 'obstacles' (ex. external boundary and defense areas) are rectangles,
     * we can model these as Rects.
     */
    Geom::Rect boundary;

} FieldBoundary;

/**
 * Creates RobotObstacles based on the friendly team robots.
 * 
 * @param friendly_team The team of friendly robots.
 * @param avoid_dist Avoidance distance around robots in meters.
 * 
 * @return Vector of all friendly robots as RobotObstacles.
 */
std::vector<RobotObstacle> process_friendly(Team& friendly_team, double avoid_dist);

/**
 * Creates RobotObstacles around the enemy team robots.
 * 
 * @param enemy_team The team of enemy robots.
 * @param avoid_dist Avoidance distance around robots in meters.
 * 
 * @return Vector of all enemy robots as RobotObstacles.
 */
std::vector<RobotObstacle> process_enemy(Team& enemy_team, double avoid_dist);

/**
 * Creates FieldBoundaries for the field 'obstacles' (ex. defense areas, outer boundaries).
 * 
 * @param field The field to get boundaries from.
 * 
 * @return Vector of relevant field boundaries as FieldBoundaries.
 */
std::vector<FieldBoundary> process_field(Field& field);

