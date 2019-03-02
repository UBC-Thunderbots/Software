#include "ai/hl/stp/evaluation/calc_best_shot.h"
#include "geom/util.h"
#include"shared/constants.h"

namespace Evaluation {

    std::pair<Point, Angle> calc_best_shot(
            const Field &f, const std::vector<Point> &obstacles,
            const Point &p, const double radius) {
        // whether that the goal is always on positive x- axis at (f.length()/2,0)?
        // player and robot are all robot type?
        // waht is mutable reference? guss it is just not const reference
        const Point p1 = Point(f.length() / 2.0, -f.goalWidth() / 2.0);
        const Point p2 = Point(f.length() / 2.0, f.goalWidth() / 2.0);
        return angleSweepCircles(
                p, p1, p2, obstacles, radius * ROBOT_MAX_RADIUS_METERS);
    }

    std::vector<std::pair<Point, Angle>> calc_best_shot_all(
            const Field &f, const std::vector<Point> &obstacles,
            const Point &p, const double radius) {

        const Point p1 = Point(f.length() / 2.0, -f.goalWidth() / 2.0);
        const Point p2 = Point(f.length() / 2.0, f.goalWidth() / 2.0);
        return angleSweepCirclesAll(
                p, p1, p2, obstacles, radius * ROBOT_MAX_RADIUS_METERS);

    }

    std::pair<Point, Angle> calc_best_shot(
            World world, Robot robot, double radius)// why not using refernce as world
    {
        std::vector<Point> obstacles;
        Team enemy = world.enemyTeam();
        Team friendly = world.friendlyTeam();
        obstacles.reserve(enemy.numRobots() + friendly.numRobots());
        for (const Robot i : enemy.getAllRobots()) {
            obstacles.push_back(i.position());
        }
        for (const Robot fpl : friendly.getAllRobots()) {
            if (fpl == robot) {
                continue;
            }
            obstacles.push_back(fpl.position());
        }
        std::pair<Point, Angle> best_shot =
                calc_best_shot(world.field(), obstacles, robot.position(), radius);
        // if there is no good shot at least make the
        // target within the goal area
        if (best_shot.second <= Angle::zero()) {
            Point temp = Point(world.field().length() / 2.0, 0.0);
            best_shot.first = temp;
        }
        return best_shot;
    }

    std::vector<std::pair<Point, Angle>> calc_best_shot_all(
            World world, Robot robot, double radius) {
        std::vector<Point> obstacles;
        Team enemy = world.enemyTeam();
        Team friendly = world.friendlyTeam();
        obstacles.reserve(enemy.numRobots() + friendly.numRobots());
        for (const Robot i : enemy.getAllRobots()) {
            obstacles.push_back(i.position());
        }
        for (const Robot fpl : friendly.getAllRobots()) {
            if (fpl == robot) {
                continue;
            }
            obstacles.push_back(fpl.position());
        }
        return calc_best_shot_all(
                world.field(), obstacles, robot.position(), radius);
    }

}
