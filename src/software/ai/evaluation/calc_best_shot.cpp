#include "software/ai/evaluation/calc_best_shot.h"

#include "software/geom/algorithms/closest_point.h"
#include "software/geom/algorithms/contains.h"

std::optional<Shot> calcBestShotOnGoal(const Segment &goal_post, const Point &shot_origin,
                                       const std::vector<Robot> &robot_obstacles,
                                       TeamType goal, double radius)
{
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

    Point shot_target = (top_point - bottom_point) / 2 + bottom_point;

    return std::make_optional(
        Shot(shot_origin, shot_target,
             Angle::fromDegrees(biggest_angle_seg.getDeltaInDegrees())));
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

std::optional<Shot> sampleForBestShotOnGoal(const Field &field, const Team &friendly_team,
                                       const Team &enemy_team,
                                       const Point &starting_point, TeamType goal,
                                       double max_dribbling_dist,
                                       int num_sample_points,
                                       const std::vector<Robot> &robots_to_ignore,
                                       double radius)
{
    std::optional<Shot> best_shot = std::nullopt;

    // Vector representing the line on which to sample shot origin points
    Vector sampling_vector =
        (field.enemyGoalCenter() - starting_point).perpendicular().normalize();

    // Spacing between sample points
    double sampling_spacing = max_dribbling_dist / (num_sample_points / 2);

    // Generate sample shot origin points offset from the starting point
    for (double sample_point_offset = -max_dribbling_dist; 
         sample_point_offset <= max_dribbling_dist; 
         sample_point_offset += sampling_spacing)
    {
        Point shot_origin = starting_point + (sampling_vector * sample_point_offset);

        if (!contains(field.fieldLines(), shot_origin) ||
            field.pointInEnemyDefenseArea(shot_origin))
        {
            continue;
        }

        std::optional<Shot> shot =
            calcBestShotOnGoal(field, friendly_team, enemy_team, shot_origin, goal,
                               robots_to_ignore, radius);

        // We consider the "best" shot to be the one with the largest open angle
        if (shot && (!best_shot || shot->getOpenAngle() > best_shot->getOpenAngle()))
        {
            best_shot = shot;
        }
    }

    return best_shot;
}
