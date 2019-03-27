#ifndef PROJECT_CALC_BEST_SHOT_H
#define PROJECT_CALC_BEST_SHOT_H

#include "ai/world/field.h"
#include "ai/world/robot.h"
#include "ai/world/world.h"
#include "geom/point.h"

namespace Evaluation
{
    /**
     * Finds the length of the largest continuous interval (angle-wise) of the
     * goal that can be seen from a point.
     *
     * @param f-field is needed to calculate the location of the goalposts based on field
     * length and goal width
     *
     * @param obstacles-obstacles is a list of all the obstacles in the way between the
     * point and the net
     *
     * @param p-the point that the shot is being calculated from
     *
     * @param radius-the radius of all obstacles
     *
     * @return the point and the score (angle),
     * where the score will be 0 if the point is invalid.
     */
    std::pair<Point, Angle> calcBestShotOnEnemyGoal(const Field &f,
                                                    const std::vector<Point> &obstacles,
                                                    const Point &p, double radius);

    /**
     * Finds the length of the all continuous interval (angle-wise) of the
     * goal that can be seen from a point.
     *
     * @param f-field is needed to calculate the location of the goalposts based on field
     * length and goal width
     *
     * @param obstacles-obstacles is a list of all the obstacles in the way between the
     * point and the net
     *
     * @param p-the point that the shot is being calculated from
     *
     * @param radius-the radius of all obstacles
     *
     * @return a set of points and the corresponding score (angle),
     * where the score will be 0 if the point is invalid.
     */
    std::vector<std::pair<Point, Angle>> calcBestShotOnEnemyGoalAll(
        const Field &f, const std::vector<Point> &obstacles, const Point &p,
        double radius);

    /**
     * Finds the length of the largest continuous interval (angle-wise) of the
     * goal that can be seen from a point.
     * To add imaginary threats or obstacles, please use the above calc_best_shot.
     *
     * @param world-world with field information
     *
     * @param point-robot that the shot is being calculated from
     *
     * @param radius-the radius of all obstacles
     *
     * @return the point as and the score (angle),
     * where the score will be 0 if the point is invalid.
     */
    std::pair<Point, Angle> calcBestShotOnEnemyGoal(const World &world,
                                                    const Point &point, double radius);


    /**
     * Finds the list of length of the largest continuous interval (angle-wise) of the
     * goal that can be seen from a point.
     * To add imaginary threats or obstacles, please use the above calc_best_shot_all.
     *
     * @param world-world with field information
     *
     * @param point-robot that the shot is being calculated from
     *
     * @@param radius-the radius of all obstacles
     *
     * @return the point as and the score (angle),
     * where the score will be 0 if the point is invalid.
     */
    std::vector<std::pair<Point, Angle>> calcBestShotOnEnemyGoalAll(const World &world,
                                                                    const Point &point,
                                                                    double radius);

}  // namespace Evaluation
#endif
