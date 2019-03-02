#include "ai\hl\stp\evaluation\calc_best_shot.h"
#include "geom\util.h"

namespace Evaluation
{

    std::pair<Point, Angle> calc_best_shot(
            const Field &f, const std::vector<Point> &obstacles,
            const Point &p, const double radius){
        // whether that the goal is always on positive x- axis
        const Point p1 = Point(f.length() / 2.0, -f.goalWidth() / 2.0);
        const Point p2 = Point(f.length() / 2.0, f.goalWidth() / 2.0);
        return angle_sweep_circles(
                p, p1, p2, obstacles, radius * Robot::MAX_RADIUS);
    }

    std::vector<std::pair<Point, Angle>> calc_best_shot_all(
            const Field &f, const std::vector<Point> &obstacles,
            const Point &p, const double radius){

        const Point p1 = Point(f.length() / 2.0, -f.goal_width() / 2.0);
        const Point p2 = Point(f.length() / 2.0, f.goal_width() / 2.0);
        return angle_sweep_circles_all(
                p, p1, p2, obstacles, radius * Robot::MAX_RADIUS);

    }

    std::pair<Point, Angle> calc_best_shot(
            World world, Robot robot, double radius)
    {
        std::vector<Point> obstacles;
        EnemyTeam enemy       = world.enemy_team();
        FriendlyTeam friendly = world.friendly_team();
        obstacles.reserve(enemy.size() + friendly.size());
        for (const Robot i : enemy)
        {
            obstacles.push_back(i.position());
        }
        for (const Player fpl : friendly)
        {
            if (fpl == player)
            {
                continue;
            }
            obstacles.push_back(fpl.position());
        }
        std::pair<Point, Angle> best_shot =
                calc_best_shot(world.field(), obstacles, player.position(), radius);
        // if there is no good shot at least make the
        // target within the goal area
        if (best_shot.second <= Angle::zero())
        {
            Point temp      = Point(world.field().length() / 2.0, 0.0);
            best_shot.first = temp;
        }
        return best_shot;
    }

    std::vector<std::pair<Point, Angle>> calc_best_shot_all(
            World world, Robot robot, double radius);
{
    std::vector<Point> obstacles;
    EnemyTeam enemy       = world.enemy_team();
    FriendlyTeam friendly = world.friendly_team();
    obstacles.reserve(enemy.size() + friendly.size());
    for (const Robot i : enemy)
{
    obstacles.push_back(i.position());
}
for (const Player fpl : friendly)
{
if (fpl == player)
{
continue;
}
obstacles.push_back(fpl.position());
}
return calc_best_shot_all(
        world.field(), obstacles, player.position(), radius);
}

}
