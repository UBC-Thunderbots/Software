#include "software/ai/evaluation/calc_best_shot.h"

std::optional<Shot> calcBestShotOnGoal(const Segment &goal_post, const Point &shot_origin,
                                       const std::vector<Robot> &robot_obstacles, double radius)
{
    Angle pos_post_angle = (goal_post.getStart() - shot_origin).orientation();
    Angle neg_post_angle = (goal_post.getEnd() - shot_origin).orientation();

    size_t max_obstacle_size = robot_obstacles.size();
    AngleMap angle_map = AngleMap(pos_post_angle, neg_post_angle, max_obstacle_size);

    std::vector<AngleSegment> obstacles;
    obstacles.reserve(max_obstacle_size);
    for (const Robot &robot_obstacle : robot_obstacles)
    {
        if (robot_obstacle.position().x() < shot_origin.x()) {
            continue;
        }

        Point enemy_robot_pos = robot_obstacle.position();
        Vector perpendicular_vec = (enemy_robot_pos - shot_origin).perpendicular();

        Vector one_end_vec = perpendicular_vec.normalize(radius);

        Point one_end = enemy_robot_pos + one_end_vec;
        Point other_end = enemy_robot_pos - one_end_vec;

        Vector top_vec = one_end - shot_origin;
        Vector bottom_vec = other_end - shot_origin;

        Angle top_angle = top_vec.orientation();
        Angle bottom_angle = bottom_vec.orientation();

        if (bottom_angle > angle_map.getAngleMax() || top_angle < angle_map.getAngleMin()) {
            continue;
        }

        obstacles.emplace_back(AngleSegment(top_angle, bottom_angle));
    }

    for (AngleSegment &obstacle : obstacles) {
        AngleSegment non_viable_angle_seg = AngleSegment(obstacle.getAngleMax(), obstacle.getAngleMin());
        angle_map.addNonViableAngleSegment(non_viable_angle_seg);
    }

    AngleSegment biggest_angle_seg = angle_map.getBiggestViableAngleSegment();
    if (biggest_angle_seg.getDelta() == 0) {
        return std::nullopt;
    }

    Angle shot_angle = ((biggest_angle_seg.getAngleMax() - biggest_angle_seg.getAngleMin()) /  2) + biggest_angle_seg.getAngleMin();
    Point shot_point = Point(4.5, (shot_angle.sin() / shot_angle.cos()) * (4.5 - shot_origin.x()) + shot_origin.y());

    Angle open_angle = Angle::fromDegrees(biggest_angle_seg.getDelta());
    return std::make_optional(Shot(shot_point, open_angle));
}

std::optional<Shot> calcBestShotOnGoal(const Field &field, const Team &friendly_team,
                                       const Team &enemy_team, const Point &shot_origin,
                                       TeamType goal,
                                       const std::vector<Robot> &robots_to_ignore, double radius)
{
    std::vector<Robot> obstacles;
    obstacles.reserve(enemy_team.numRobots() + friendly_team.numRobots() - 1);
    for (const Robot &enemy_robot : enemy_team.getAllRobots())
    {
        // Only add the robot to the obstacles if it is not ignored
        if (std::count(robots_to_ignore.begin(), robots_to_ignore.end(), enemy_robot) ==
            0)
        {
            obstacles.emplace_back(enemy_robot);
        }
    }
    for (const Robot &friendly_robot : friendly_team.getAllRobots())
    {
        // Only add the robot to the obstacles if it is not ignored
        if (std::count(robots_to_ignore.begin(), robots_to_ignore.end(),
                       friendly_robot) == 0)
        {
            obstacles.emplace_back(friendly_robot);
        }
    }

    if (goal == TeamType::FRIENDLY)
    {
        return calcBestShotOnGoal(Segment(field.friendlyGoalpostPos(), field.friendlyGoalpostNeg()), shot_origin, obstacles);
    }
    else
    {
        return calcBestShotOnGoal(Segment(field.enemyGoalpostPos(), field.enemyGoalpostNeg()), shot_origin, obstacles);
    }
}
