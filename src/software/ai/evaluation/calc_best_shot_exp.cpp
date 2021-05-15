#include <optional>
#include "calc_best_shot_exp.h"
#include "software/geom/ray.h"
#include "software/geom/algorithms/intersection.h"



std::optional<Shot> calcBestShotOnGoalExp(const Segment &goal_post, const Point &shot_origin,
                                          const std::vector<Robot> &robot_obstacles) {
    return std::nullopt;
}

std::optional<Shot> calcBestShotOnGoalExp(const Field &field, const Team &friendly_team,
                                       const Team &enemy_team, const Point &shot_origin,
                                       TeamType goal,
                                       const std::vector<Robot> &robots_to_ignore) {

    Segment goal_seg = Segment(field.enemyGoalpostPos(), field.enemyGoalpostNeg());
    Angle pos_post_angle = (goal_seg.getStart() - shot_origin).orientation();
    Angle neg_post_angle = (goal_seg.getEnd() - shot_origin).orientation();

    size_t max_obstacle_size = enemy_team.numRobots() + friendly_team.numRobots() - 1;
    AngleMap angle_map = AngleMap(pos_post_angle, neg_post_angle, max_obstacle_size);

    std::vector<AngleSegment> obstacles;
    obstacles.reserve(max_obstacle_size);
    for (const Robot &enemy_robot : enemy_team.getAllRobots())
    {
        // Only add the robot to the obstacles if it is not ignored
        if (std::count(robots_to_ignore.begin(), robots_to_ignore.end(), enemy_robot) ==
            0)
        {
            if (enemy_robot.position().x() < shot_origin.x()) {
                continue;
            }

            Point enemy_robot_pos = enemy_robot.position();
            Vector perpendicular_vec = (enemy_robot_pos - shot_origin).perpendicular();

            Vector one_end_vec = perpendicular_vec.normalize(ROBOT_MAX_RADIUS_METERS);

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
    }
    for (const Robot &friendly_robot : friendly_team.getAllRobots())
    {
        // Only add the robot to the obstacles if it is not ignored
        if (std::count(robots_to_ignore.begin(), robots_to_ignore.end(),
                       friendly_robot) == 0)
        {
            if (friendly_robot.position().x() < shot_origin.x()) {
                continue;
            }

            Point enemy_robot_pos = friendly_robot.position();
            Vector perpendicular_vec = (enemy_robot_pos - shot_origin).perpendicular();

            Vector one_end_vec = perpendicular_vec.normalize(ROBOT_MAX_RADIUS_METERS);

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
