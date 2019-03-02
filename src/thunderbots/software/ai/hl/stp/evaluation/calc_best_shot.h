#ifndef PROJECT_CALC_BEST_SHOT_H
#define PROJECT_CALC_BEST_SHOT_H

#include "geom/point.h"
#include "ai/world/field.h"
#include "ai/world/world.h"
#include "ai/world/robot.h"

namespace Evaluation
{
    /**
 * Finds the length of the largest continuous interval (angle-wise) of the enemy
 * goal that can be seen from a point.
 * Having a vector of points enables one to add imaginary threats.
 * This version accepts vector of obstacles, so that you can add imaginary
 * robots.
 *
 * \param[in] f field is needed to calculate length etc
 *
 * \param[in] radius the multiplier to the radius of the robot,
 * you can decrease the radius to make it easier to shoot.
 *
 * \param[in] obstacles is a list of all the obstacles in the way between the
 * robot and the net
 *
 * \param[in] p the player that the shot is being calculated from
 *
 * \return the point and the score (angle),
 * where the score will be 0 if the point is invalid.
 */
    std::pair<Point, Angle> calc_best_shot(
            const Field &f, const std::vector<Point> &obstacles,
            const Point &p, double radius = 1.0);

    std::vector<std::pair<Point, Angle>> calc_best_shot_all(
            const Field &f, const std::vector<Point> &obstacles,
            const Point &p, double radius = 1.0);

/**
 * Finds the length of the largest continuous interval (angle-wise) of the enemy
 * goal that can be seen from a point.
 * To add imaginary threats, please use the other version.
 *
 * \param[in] world with field information
 *
 * \param[in] player player that the shot is being calculated from
 *
 * \param[in] radius the multiplier to the radius of the robot,
 * you can decrease the radius to make it easier to shoot.
 *
 * \return the point as and the score (angle),
 * where the score will be 0 if the point is invalid,
 */
    std::pair<Point, Angle> calc_best_shot(
            World world, Robot robot, double radius = 1.0);

    std::vector<std::pair<Point, Angle>> calc_best_shot_all(
            World world, Robot robot, double radius = 1.0);

}
#endif

