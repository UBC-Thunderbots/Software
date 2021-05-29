#include "software/ai/evaluation/calc_best_shot.h"

std::optional<Shot> calcBestShotOnGoal(const Segment &goal_post, const Point &shot_origin,
                                       const std::vector<Robot> &robot_obstacles,
                                       TeamType goal, double radius)
{
    std::optional<AngleMap> angle_map;

    Angle pos_post_angle     = (goal_post.getStart() - shot_origin).orientation();
    Angle neg_post_angle     = (goal_post.getEnd() - shot_origin).orientation();
    size_t max_obstacle_size = robot_obstacles.size();

    std::vector<ObstacleAngleSegment> obstacles;
    obstacles.reserve(max_obstacle_size);

    if (goal == TeamType::ENEMY)
    {
        angle_map = AngleMap(pos_post_angle, neg_post_angle, max_obstacle_size);

        for (const Robot &robot_obstacle : robot_obstacles)
        {
            Point enemy_robot_pos    = robot_obstacle.position();
            Vector perpendicular_vec = (enemy_robot_pos - shot_origin).perpendicular();

            Vector one_end_vec = perpendicular_vec.normalize(radius);

            Point one_end   = enemy_robot_pos + one_end_vec;
            Point other_end = enemy_robot_pos - one_end_vec;

            Vector top_vec    = one_end - shot_origin;
            Vector bottom_vec = other_end - shot_origin;

            Angle top_angle    = top_vec.orientation();
            Angle bottom_angle = bottom_vec.orientation();

            if (bottom_angle > angle_map->getAngleSegment().getAngleTop() ||
                top_angle < angle_map->getAngleSegment().getAngleBottom())
            {
                continue;
            }

            ObstacleAngleSegment non_viable_angle_seg =
                ObstacleAngleSegment(top_angle, bottom_angle);
            angle_map->addObstacleAngleSegment(non_viable_angle_seg);
        }
    }
    else
    {
        if (pos_post_angle.toRadians() < 0)
        {
            pos_post_angle += Angle::fromRadians(2 * M_PI);
        }
        if (neg_post_angle.toRadians() < 0)
        {
            neg_post_angle += Angle::fromRadians(2 * M_PI);
        }

        angle_map = AngleMap(neg_post_angle, pos_post_angle, max_obstacle_size);

        for (const Robot &robot_obstacle : robot_obstacles)
        {
            Point enemy_robot_pos    = robot_obstacle.position();
            Vector perpendicular_vec = (enemy_robot_pos - shot_origin).perpendicular();

            Vector one_end_vec = perpendicular_vec.normalize(radius);

            Point one_end   = enemy_robot_pos + one_end_vec;
            Point other_end = enemy_robot_pos - one_end_vec;

            Vector top_vec    = one_end - shot_origin;
            Vector bottom_vec = other_end - shot_origin;

            Angle top_angle = top_vec.orientation();
            if (top_angle.toRadians() < 0)
            {
                top_angle += Angle::fromRadians(2 * M_PI);
            }

            Angle bottom_angle = bottom_vec.orientation();
            if (bottom_angle.toRadians() < 0)
            {
                bottom_angle += Angle::fromRadians(2 * M_PI);
            }

            if (bottom_angle > angle_map->getAngleSegment().getAngleTop() ||
                top_angle < angle_map->getAngleSegment().getAngleBottom())
            {
                continue;
            }

            ObstacleAngleSegment non_viable_angle_seg =
                ObstacleAngleSegment(top_angle, bottom_angle);
            angle_map->addObstacleAngleSegment(non_viable_angle_seg);
        }
    }

    AngleSegment biggest_angle_seg = angle_map->getBiggestViableAngleSegment();
    if (biggest_angle_seg.getDelta() == 0)
    {
        return std::nullopt;
    }

    Angle top_angle = biggest_angle_seg.getAngleTop();
    if (goal == TeamType::FRIENDLY)
    {
        if (top_angle.toRadians() > M_PI)
        {
            top_angle -= Angle::fromRadians(2 * M_PI);
        }
    }

    Angle bottom_angle = biggest_angle_seg.getAngleBottom();
    if (goal == TeamType::FRIENDLY)
    {
        if (bottom_angle.toRadians() > M_PI)
        {
            bottom_angle -= Angle::fromRadians(2 * M_PI);
        }
    }

    Point top_point    = Point(goal_post.getStart().x(),
                            (top_angle.sin() / top_angle.cos()) *
                                    (goal_post.getStart().x() - shot_origin.x()) +
                                shot_origin.y());
    Point bottom_point = Point(goal_post.getStart().x(),
                               (bottom_angle.sin() / bottom_angle.cos()) *
                                       (goal_post.getStart().x() - shot_origin.x()) +
                                   shot_origin.y());

    Point shot_point = (top_point - bottom_point) / 2 + bottom_point;

    Angle open_angle = Angle::fromDegrees(biggest_angle_seg.getDelta());
    return std::make_optional(Shot(shot_point, open_angle));
}

std::optional<Shot> calcBestShotOnGoal(const Field &field, const Team &friendly_team,
                                       const Team &enemy_team, const Point &shot_origin,
                                       TeamType goal,
                                       const std::vector<Robot> &robots_to_ignore,
                                       double radius)
{
    std::vector<Robot> obstacles;
    obstacles.reserve(enemy_team.numRobots() + friendly_team.numRobots() - 1);
    for (const Robot &enemy_robot : enemy_team.getAllRobots())
    {
        // Only add the robot to the obstacles if it is not ignored
        if (std::count(robots_to_ignore.begin(), robots_to_ignore.end(), enemy_robot) ==
            0)
        {
            if (goal == TeamType::ENEMY)
            {
                if (enemy_robot.position().x() < shot_origin.x())
                {
                    continue;
                }
            }
            else
            {
                if (enemy_robot.position().x() > shot_origin.x())
                {
                    continue;
                }
            }

            obstacles.emplace_back(enemy_robot);
        }
    }
    for (const Robot &friendly_robot : friendly_team.getAllRobots())
    {
        // Only add the robot to the obstacles if it is not ignored
        if (std::count(robots_to_ignore.begin(), robots_to_ignore.end(),
                       friendly_robot) == 0)
        {
            if (goal == TeamType::ENEMY)
            {
                if (friendly_robot.position().x() < shot_origin.x())
                {
                    continue;
                }
            }
            else
            {
                if (friendly_robot.position().x() > shot_origin.x())
                {
                    continue;
                }
            }

            obstacles.emplace_back(friendly_robot);
        }
    }

    if (goal == TeamType::FRIENDLY)
    {
        return calcBestShotOnGoal(
            Segment(field.friendlyGoalpostPos(), field.friendlyGoalpostNeg()),
            shot_origin, obstacles, goal, radius);
    }
    else
    {
        return calcBestShotOnGoal(
            Segment(field.enemyGoalpostPos(), field.enemyGoalpostNeg()), shot_origin,
            obstacles, goal, radius);
    }
}
