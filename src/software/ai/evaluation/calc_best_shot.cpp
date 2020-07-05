#include "software/ai/evaluation/calc_best_shot.h"

#include "software/geom/util.h"
#include "software/new_geom/util/acute_angle.h"
#include "software/new_geom/util/multiple_segments.h"
#include "software/new_geom/util/projection.h"

std::optional<Shot> calcBestShotOnGoal(const Point &goal_post_neg,
                                       const Point &goal_post_pos, const Point &p,
                                       const std::vector<Point> &obstacles, double radius)
{
    // Use shot evaluation function to get the best Shot
    std::vector<Circle> obs;
    for (Point point : obstacles)
    {
        obs.push_back(Circle(point, radius));
    }
    return calcMostOpenDirectionFromCircleObstacles(
        p, Segment(goal_post_neg, goal_post_pos), obs);
}
std::optional<Shot> calcBestShotOnGoal(const Point &goal_post_neg,
                                       const Point &goal_post_pos, const Point &p,
                                       const std::vector<Robot> &robot_obstacles)
{
    // Use shot evaluation function to get the best Shot
    std::vector<Circle> obs;
    for (Robot robot : robot_obstacles)
    {
        obs.push_back(Circle(robot.position(), ROBOT_MAX_RADIUS_METERS));
    }
    return calcMostOpenDirectionFromCircleObstacles(
        p, Segment(goal_post_neg, goal_post_pos), obs);
}
std::optional<Shot> calcBestShotOnGoal(const Point &goal_post_neg,
                                       const Point &goal_post_pos, const Point &p,
                                       const std::vector<Circle> &obstacles)
{
    // Use shot evaluation function to get the best Shot
    return calcMostOpenDirectionFromCircleObstacles(
        p, Segment(goal_post_neg, goal_post_pos), obstacles);
}

std::optional<Shot> calcBestShotOnGoal(const Field &field, const Team &friendly_team,
                                       const Team &enemy_team, const Point &point,
                                       bool shoot_on_enemy_goal, double radius,
                                       const std::vector<Robot> &robots_to_ignore)
{
    std::vector<Circle> obstacles;
    for (const Robot &enemy_robot : enemy_team.getAllRobots())
    {
        // Only add the robot to the obstacles if it is not ignored
        if (std::count(robots_to_ignore.begin(), robots_to_ignore.end(), enemy_robot) ==
            0)
        {
            obstacles.emplace_back(Circle(enemy_robot.position(), radius));
        }
    }
    for (const Robot &friendly_robot : friendly_team.getAllRobots())
    {
        // Only add the robot to the obstacles if it is not ignored
        if (std::count(robots_to_ignore.begin(), robots_to_ignore.end(),
                       friendly_robot) == 0)
        {
            obstacles.emplace_back(Circle(friendly_robot.position(), radius));
        }
    }

    std::optional<Shot> best_shot;

    // Calculate the best_shot based on what goal we're shooting at
    if (shoot_on_enemy_goal)
    {
        best_shot = calcBestShotOnGoal(field.enemyGoalpostNeg(), field.enemyGoalpostPos(),
                                       point, obstacles);
    }
    else
    {
        best_shot = calcBestShotOnGoal(field.friendlyGoalpostNeg(),
                                       field.friendlyGoalpostPos(), point, obstacles);
    }

    return best_shot;
}

std::optional<Shot> calcBestShotOnEnemyGoal(const Field &field, const Team &friendly_team,
                                            const Team &enemy_team, const Robot &robot,
                                            double radius,
                                            const std::vector<Robot> &robots_to_ignore)
{
    std::vector<Robot> all_robots_to_ignore = robots_to_ignore;
    // Ignore the robot shooting the ball
    all_robots_to_ignore.emplace_back(robot);
    return calcBestShotOnEnemyGoal(field, friendly_team, enemy_team, robot.position(),
                                   radius, all_robots_to_ignore);
}

std::optional<Shot> calcBestShotOnEnemyGoal(const Field &field, const Team &friendly_team,
                                            const Team &enemy_team,
                                            const Point &shot_origin, double radius,
                                            const std::vector<Robot> &robots_to_ignore)
{
    return calcBestShotOnGoal(field, friendly_team, enemy_team, shot_origin, true, radius,
                              robots_to_ignore);
}

std::optional<Shot> calcBestShotOnFriendlyGoal(const Field &field,
                                               const Team &friendly_team,
                                               const Team &enemy_team, const Robot &robot,
                                               double radius,
                                               const std::vector<Robot> &robots_to_ignore)
{
    std::vector<Robot> all_robots_to_ignore = robots_to_ignore;
    // Ignore the robot shooting the ball
    all_robots_to_ignore.emplace_back(robot);
    return calcBestShotOnFriendlyGoal(field, friendly_team, enemy_team, robot.position(),
                                      radius, all_robots_to_ignore);
}

std::optional<Shot> calcBestShotOnFriendlyGoal(const Field &field,
                                               const Team &friendly_team,
                                               const Team &enemy_team,
                                               const Point &shot_origin, double radius,
                                               const std::vector<Robot> &robots_to_ignore)
{
    return calcBestShotOnGoal(field, friendly_team, enemy_team, shot_origin, false,
                              radius, robots_to_ignore);
}

double calcShotOpenFriendlyNetPercentage(const Field &field, const Point &shot_origin,
                                         const Shot &shot)
{
    Angle goal_angle =
        acuteAngle(field.friendlyGoalpostPos(), shot_origin, field.friendlyGoalpostNeg())
            .abs();
    return shot.getOpenAngle().toDegrees() / goal_angle.toDegrees();
}

double calcShotOpenEnemyNetPercentage(const Field &field, const Point &shot_origin,
                                      const Shot &shot)
{
    Angle goal_angle =
        acuteAngle(field.enemyGoalpostPos(), shot_origin, field.enemyGoalpostNeg()).abs();
    return shot.getOpenAngle().toDegrees() / goal_angle.toDegrees();
}

std::optional<Shot> calcMostOpenDirectionFromRobotObstacles(
    Point origin, Segment segment, std::vector<Robot> robot_obstacles)
{
    std::vector<Circle> obstacles;

    for (Robot robot : robot_obstacles)
    {
        obstacles.push_back(Circle(robot.position(), ROBOT_MAX_RADIUS_METERS));
    }

    return calcMostOpenDirectionFromCircleObstacles(origin, segment, obstacles);
}
std::optional<Shot> calcMostOpenDirectionFromCircleObstacles(
    Point origin, Segment segment, std::vector<Circle> obstacles)
{
    std::vector<Segment> obstacle_segment_projections;

    // If there are no obstacles, return the center of the Segment and the shot angle
    if (obstacles.size() == 0)
    {
        const Point center_of_segment =
            getPointsMean({segment.getSegStart(), segment.getEnd()});
        const Angle angle_of_entire_segment =
            ((segment.getSegStart() - origin)
                 .orientation()
                 .minDiff((segment.getEnd() - origin).orientation()))
                .abs();

        return std::make_optional(Shot(center_of_segment, angle_of_entire_segment));
    }

    obstacle_segment_projections = projectCirclesOntoSegment(segment, obstacles, origin);

    // If we have more than 1 Segment from the obstacle projection then we must
    // combine overlapping ones to simplify analysis
    if (obstacle_segment_projections.size() > 1)
    {
        obstacle_segment_projections =
            realignSegmentsOntoVector(obstacle_segment_projections,
                                      obstacle_segment_projections.front().toVector());
    }
    else if (obstacle_segment_projections.size() == 0)
    {
        // If there are no blocking Segments, just shoot at the center of the goal
        const Point center_of_segment =
            getPointsMean({segment.getSegStart(), segment.getEnd()});
        const Angle angle_of_entire_segment =
            ((segment.getSegStart() - origin)
                 .orientation()
                 .minDiff((segment.getEnd() - origin).orientation()))
                .abs();

        return std::make_optional(Shot(center_of_segment, angle_of_entire_segment));
    }
    std::vector<Segment> open_segs;

    open_segs = getEmptySpaceWithinParentSegment(obstacle_segment_projections, segment);

    Segment largest_segment;

    if (open_segs.size() >= 2)
    {
        largest_segment = *std::max_element(open_segs.begin(), open_segs.end(),
                                            [](const Segment &s1, const Segment &s2) {
                                                return s1.length() < s2.length();
                                            });
    }
    else if (open_segs.size() == 1)
    {
        largest_segment = open_segs.front();
    }
    else
    {
        return std::nullopt;
    }


    const Point most_open_point =
        Point((largest_segment.getSegStart().x() + largest_segment.getEnd().x()) / 2,
              (largest_segment.getSegStart().y() + largest_segment.getEnd().y()) / 2);
    const Angle largest_open_angle =
        ((largest_segment.getSegStart() - origin)
             .orientation()
             .minDiff((largest_segment.getEnd() - origin).orientation()))
            .abs();

    return std::make_optional(Shot(most_open_point, largest_open_angle));
}
