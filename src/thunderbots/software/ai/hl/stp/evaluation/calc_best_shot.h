#pragma once

#include "ai/world/field.h"
#include "ai/world/robot.h"
#include "ai/world/world.h"
#include "geom/point.h"

namespace Evaluation
{
    /**
     * Finds the best shot on the given goal, and returns the best target to shoot at and
     * the largest open angle interval for the shot (this is the total angle between the
     * obstacles on either side of the shot vector).
     *
     * @param goal_post_neg The goal post of the net with the negative y-coordinate
     * @param goal_post_pos The goal post of the net with the positive y-coordinate
     * @param p The point that the shot will be taken from
     * @param obstacles The locations of any obstacles on the field that may obstruct the
     * shot. These are treated as circular obstacles, and are usually used to represent
     * robots on the field.
     * @param radius The radius of the given obstacles
     *
     * @return the best target to shoot at and the largest open angle interval for the
     * shot (this is the total angle between the obstacles on either side of the shot
     * vector). If no shot is possible, the returned angle will be 0
     */
    std::pair<Point, Angle> calcBestShotOnGoal(const Point &goal_post_neg,
                                               const Point &goal_post_pos, const Point &p,
                                               const std::vector<Point> &obstacles,
                                               double radius);

    /**
     * Finds the best shot on the specified goal, and returns the best target to shoot at
     * and the largest open angle interval for the shot (this is the total angle between
     * the obstacles on either side of the shot vector).
     *
     * This function will treat all robots on the field as obstacles, except for the
     * robots_to_ignore
     *
     * @param world The world state
     * @param point The point that the shot will be taken from
     * @param radius The radius for the robot obstacles
     * @param robots_to_ignore The robots to ignore
     * @param shoot_at_enemy_goal Whether or not the shot is at the enemy goal. If true,
     * the shot will be treated as a shot at the enemy goal, and if false, the shot will
     * be treated as a shot at the friendly goal
     *
     * @return the best target to shoot at and the largest open angle interval for the
     * shot (this is the total angle between the obstacles on either side of the shot
     * vector).
     */
    std::pair<Point, Angle> calcBestShotOnGoal(const World &world, const Point &point,
                                               double radius,
                                               const std::vector<Robot> &robots_to_ignore,
                                               bool shoot_at_enemy_goal);

    /**
     * Finds the best shot on the enemy goal, and returns the best target to shoot at and
     * the largest open angle interval for the shot (this is the total angle between the
     * obstacles on either side of the shot vector).
     *
     * This function will treat all robot on the field as obstacles, except for the robot
     * taking the shot and the robots_to_ignore
     *
     * @param world The world state
     * @param robot The robot taking the shot
     * @param radius The radius for the robot obstacles
     * @param robots_to_ignore The robots to ignore
     *
     * @return the best target to shoot at and the largest open angle interval for the
     * shot (this is the total angle between the obstacles on either side of the shot
     * vector).
     */
    std::pair<Point, Angle> calcBestShotOnEnemyGoal(
        const World &world, const Robot &robot, double radius,
        const std::vector<Robot> &robots_to_ignore = {});

    /**
     * Finds the best shot on the enemy goal, and returns the best target to shoot at and
     * the largest open angle interval for the shot (this is the total angle between the
     * obstacles on either side of the shot vector).
     *
     * This function will treat all robot on the field as obstacles, except for the
     * robots_to_ignore
     *
     * @param world The world state
     * @param shot_origin The point the shot is being taken from
     * @param radius The radius for the robot obstacles
     * @param robots_to_ignore The robots to ignore
     *
     * @return the best target to shoot at and the largest open angle interval for the
     * shot (this is the total angle between the obstacles on either side of the shot
     * vector).
     */
    std::pair<Point, Angle> calcBestShotOnEnemyGoal(
        const World &world, const Point &shot_origin, double radius,
        const std::vector<Robot> &robots_to_ignore = {});

    /**
     * Finds the best shot on the friendly goal, and returns the best target to shoot at
     * and the largest open angle interval for the shot (this is the total angle between
     * the obstacles on either side of the shot vector).
     *
     * This function will treat all robot on the field as obstacles, except for the robot
     * taking the shot and the robots_to_ignore
     *
     * @param world The world state
     * @param robot The robot taking the shot
     * @param radius The radius for the robot obstacles
     * @param robots_to_ignore The robots to ignore
     *
     * @return the best target to shoot at and the largest open angle interval for the
     * shot (this is the total angle between the obstacles on either side of the shot
     * vector).
     */
    std::pair<Point, Angle> calcBestShotOnFriendlyGoal(
        const World &world, const Robot &robot, double radius,
        const std::vector<Robot> &robots_to_ignore = {});

    /**
     * Finds the best shot on the friendly goal, and returns the best target to shoot at
     * and the largest open angle interval for the shot (this is the total angle between
     * the obstacles on either side of the shot vector).
     *
     * This function will treat all robot on the field as obstacles, except for the
     * robots_to_ignore
     *
     * @param world The world state
     * @param shot_origin The point the shot is being taken from
     * @param radius The radius for the robot obstacles
     * @param robots_to_ignore The robots to ignore
     *
     * @return the best target to shoot at and the largest open angle interval for the
     * shot (this is the total angle between the obstacles on either side of the shot
     * vector).
     */
    std::pair<Point, Angle> calcBestShotOnFriendlyGoal(
        const World &world, const Point &shot_origin, double radius,
        const std::vector<Robot> &robots_to_ignore = {});
}  // namespace Evaluation
