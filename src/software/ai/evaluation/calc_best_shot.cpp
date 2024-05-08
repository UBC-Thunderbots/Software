#include "software/ai/evaluation/calc_best_shot.h"

std::optional<Shot> calcBestShotOnGoal(const Segment &goal_post, const Point &shot_origin,
                                       const std::vector<Robot> &robot_obstacles,
                                       TeamType goal, double radius)
{
    // Don't return a shot if the ball is behind the net
    if ((goal == TeamType::FRIENDLY && shot_origin.x() < goal_post.getStart().x()) ||
        (goal == TeamType::ENEMY && shot_origin.x() > goal_post.getStart().x()))
    {
        return std::nullopt;
    }

    size_t max_num_obstacles = robot_obstacles.size();

    Angle pos_post_angle = (goal_post.getStart() - shot_origin).orientation();
    Angle neg_post_angle = (goal_post.getEnd() - shot_origin).orientation();

    std::vector<AngleSegment> obstacles;
    obstacles.reserve(max_num_obstacles);

    if (goal == TeamType::FRIENDLY)
    {
        auto tmp       = pos_post_angle;
        pos_post_angle = (neg_post_angle + Angle::half()).clamp();
        neg_post_angle = (tmp + Angle::half()).clamp();
    }
    AngleMap angle_map(pos_post_angle, neg_post_angle, max_num_obstacles);

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
        if (goal == TeamType::FRIENDLY)
        {
            top_angle    = (top_vec.orientation() + Angle::half()).clamp();
            bottom_angle = (bottom_vec.orientation() + Angle::half()).clamp();
        }

        if (bottom_angle > angle_map.getAngleSegment().getAngleTop() ||
            top_angle < angle_map.getAngleSegment().getAngleBottom())
        {
            continue;
        }

        AngleSegment non_viable_angle_seg = AngleSegment(top_angle, bottom_angle);
        obstacles.emplace_back(non_viable_angle_seg);
    }

    std::sort(obstacles.begin(), obstacles.end(),
              [](AngleSegment &a, AngleSegment &b) -> bool { return a > b; });

    for (AngleSegment &obstacle_angle_seg : obstacles)
    {
        angle_map.addNonViableAngleSegment(obstacle_angle_seg);
    }

    AngleSegment biggest_angle_seg = angle_map.getBiggestViableAngleSegment();
    if (biggest_angle_seg.getDeltaInDegrees() == 0)
    {
        return std::nullopt;
    }

    Angle top_angle    = biggest_angle_seg.getAngleTop();
    Angle bottom_angle = biggest_angle_seg.getAngleBottom();

    if (goal == TeamType::FRIENDLY)
    {
        top_angle    = (top_angle + Angle::half()).clamp();
        bottom_angle = (bottom_angle + Angle::half()).clamp();
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

    return std::make_optional(
        Shot(shot_point, Angle::fromDegrees(biggest_angle_seg.getDeltaInDegrees())));
}

std::optional<Shot> calcBestShotOnGoal(const Field &field, const Team &friendly_team,
                                       const Team &enemy_team, const Point &shot_origin,
                                       TeamType goal,
                                       const std::vector<Robot> &robots_to_ignore,
                                       double radius)
{
    if (shot_origin.x() < field.friendlyGoalCenter().x() ||
        shot_origin.x() > field.enemyGoalCenter().x())
    {
        return std::nullopt;
    }

    std::vector<Robot> obstacles;
    std::vector<Robot> all_robots;

    size_t max_num_robots = enemy_team.numRobots() + friendly_team.numRobots();
    all_robots.reserve(max_num_robots);
    all_robots.insert(all_robots.begin(), enemy_team.getAllRobots().begin(),
                      enemy_team.getAllRobots().end());
    all_robots.insert(all_robots.begin(), friendly_team.getAllRobots().begin(),
                      friendly_team.getAllRobots().end());

    for (const Robot &robot : all_robots)
    {
        if (std::count(robots_to_ignore.begin(), robots_to_ignore.end(), robot) == 0)
        {
            if (goal == TeamType::ENEMY)
            {
                if (robot.position().x() < shot_origin.x())
                {
                    continue;
                }
            }
            else
            {
                if (robot.position().x() > shot_origin.x())
                {
                    continue;
                }
            }

            obstacles.emplace_back(robot);
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
