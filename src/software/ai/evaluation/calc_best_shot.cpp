#include "software/ai/evaluation/calc_best_shot.h"

#include "software/geom/util.h"

namespace Evaluation
{
    std::optional<Shot> calcBestShotOnGoal(const Point &goal_post_neg,
                                           const Point &goal_post_pos, const Point &p,
                                           const std::vector<Point> &obstacles,
                                           double radius)
    {
        // Use shot evaluation function to get the best Shot
        std::vector<Circle> obs;
        for (Point point : obstacles)
        {
            obs.push_back(Circle(point, radius));
        }
        return std::make_optional(
            calcMostOpenDirection(p, Segment(goal_post_neg, goal_post_pos), obs));
    }
    std::optional<Shot> calcBestShotOnGoal(const Point &goal_post_neg,
                                           const Point &goal_post_pos, const Point &p,
                                           const std::vector<Robot> &robot_obstacles
                                           )
    {
        // Use shot evaluation function to get the best Shot
        std::vector<Circle> obs;
        for (Robot robot : robot_obstacles)
        {
            obs.push_back(Circle(robot.position(), ROBOT_MAX_RADIUS_METERS));
        }
        return std::make_optional(
                calcMostOpenDirection(p, Segment(goal_post_neg, goal_post_pos), obs));
    }
    std::optional<Shot> calcBestShotOnGoal(const Point &goal_post_neg,
                                           const Point &goal_post_pos, const Point &p,
                                           const std::vector<Circle> &obstacles
    )
    {
        // Use shot evaluation function to get the best Shot
        return std::make_optional(
                calcMostOpenDirection(p, Segment(goal_post_neg, goal_post_pos), obstacles));
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
            if (std::count(robots_to_ignore.begin(), robots_to_ignore.end(),
                           enemy_robot) == 0)
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
            best_shot = calcBestShotOnGoal(field.enemyGoalpostNeg(), field.enemyGoalpostPos(), point, obstacles);
        }
        else
        {
            best_shot = calcBestShotOnGoal(field.friendlyGoalpostNeg(), field.friendlyGoalpostPos(), point, obstacles);
        }

        return best_shot;
    }

    std::optional<Shot> calcBestShotOnEnemyGoal(
        const Field &field, const Team &friendly_team, const Team &enemy_team,
        const Robot &robot, double radius, const std::vector<Robot> &robots_to_ignore)
    {
        std::vector<Robot> all_robots_to_ignore = robots_to_ignore;
        // Ignore the robot shooting the ball
        all_robots_to_ignore.emplace_back(robot);
        return calcBestShotOnEnemyGoal(field, friendly_team, enemy_team, robot.position(),
                                       radius, all_robots_to_ignore);
    }

    std::optional<Shot> calcBestShotOnEnemyGoal(
        const Field &field, const Team &friendly_team, const Team &enemy_team,
        const Point &shot_origin, double radius,
        const std::vector<Robot> &robots_to_ignore)
    {
        return calcBestShotOnGoal(field, friendly_team, enemy_team, shot_origin, true,
                                  radius, robots_to_ignore);
    }

    std::optional<Shot> calcBestShotOnFriendlyGoal(
        const Field &field, const Team &friendly_team, const Team &enemy_team,
        const Robot &robot, double radius, const std::vector<Robot> &robots_to_ignore)
    {
        std::vector<Robot> all_robots_to_ignore = robots_to_ignore;
        // Ignore the robot shooting the ball
        all_robots_to_ignore.emplace_back(robot);
        return calcBestShotOnFriendlyGoal(field, friendly_team, enemy_team,
                                          robot.position(), radius, all_robots_to_ignore);
    }

    std::optional<Shot> calcBestShotOnFriendlyGoal(
        const Field &field, const Team &friendly_team, const Team &enemy_team,
        const Point &shot_origin, double radius,
        const std::vector<Robot> &robots_to_ignore)
    {
        return calcBestShotOnGoal(field, friendly_team, enemy_team, shot_origin, false,
                                  radius, robots_to_ignore);
    }

    double calcShotOpenFriendlyNetPercentage(const Field &field, const Point &shot_origin,
                                             const Shot &shot)
    {
        Angle goal_angle = acuteVertexAngle(field.friendlyGoalpostPos(), shot_origin,
                                            field.friendlyGoalpostNeg())
                               .abs();
        return shot.getOpenAngle().toDegrees() / goal_angle.toDegrees();
    }

    double calcShotOpenEnemyNetPercentage(const Field &field, const Point &shot_origin,
                                          const Shot &shot)
    {
        Angle goal_angle = acuteVertexAngle(field.enemyGoalpostPos(), shot_origin,
                                            field.enemyGoalpostNeg())
                               .abs();
        return shot.getOpenAngle().toDegrees() / goal_angle.toDegrees();
    }

    Shot calcMostOpenDirection(Point origin, Segment segment,
                               std::vector<Robot> robot_obstacles)
    {
        std::vector<Circle> obstacles;

        for (Robot robot : robot_obstacles)
        {
            obstacles.push_back(Circle(robot.position(), ROBOT_MAX_RADIUS_METERS));
        }

        return calcMostOpenDirection(origin, segment, obstacles);
    }
    Shot calcMostOpenDirection(Point origin, Segment segment,
                               std::vector<Circle> obstacles)
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

            return Shot(center_of_segment, angle_of_entire_segment);
        }

        // Loop through all obstacles to create their 'blocking' Segment
        for (Circle circle : obstacles)
        {
            // If the reference is inside an obstacle there is no open direction
            if (contains(circle, origin))
            {
                const Point center_of_segment =
                    getPointsMean({segment.getSegStart(), segment.getEnd()});
                return Shot(center_of_segment, Angle::fromDegrees(0));
            }

            // Get the tangent rays from the reference point to the obstacle
            auto [ray1, ray2] = getCircleTangentRaysWithReferenceOrigin(origin, circle);

            // Project the tangent Rays to obtain a 'blocked' segment on the reference
            // Segment
            std::optional<Segment> intersect_segment =
                getIntersectingSegment(ray1, ray2, segment);

            if (intersect_segment.has_value())
            {
                obstacle_segment_projections.push_back(intersect_segment.value());
            }
        }

        // If we have more than 1 Segment from the obstacle projection then we must
        // combine overlapping ones to simplify analysis
        if (obstacle_segment_projections.size() >= 2)
        {
            obstacle_segment_projections =
                reduceParallelSegments(obstacle_segment_projections);
        }
        // Make sure the starting point of all segments is closer to the start of the
        // reference segment to simplify the evaluation
        for (auto &unordered_seg : obstacle_segment_projections)
        {
            if ((segment.getSegStart() - unordered_seg.getSegStart()).length() >
                (segment.getSegStart() - unordered_seg.getEnd()).length())
            {
                // We need to flip the start/end of the segment
                Segment temp = unordered_seg;
                unordered_seg.setSegStart(temp.getEnd());
                unordered_seg.setEnd(temp.getSegStart());
            }
        }

        // Now we must sort the segments so that we can iterate through them in order to
        // generate open angles sort using a lambda expression
        // We sort the segments based on how close their 'start' point is to the 'start'
        // of the reference Segment
        std::sort(obstacle_segment_projections.begin(),
                  obstacle_segment_projections.end(), [segment](Segment &a, Segment &b) {
                      return (segment.getSegStart() - a.getSegStart()).length() <
                             (segment.getSegStart() - b.getSegStart()).length();
                  });

        // Now we need to find the largest open segment/angle
        std::vector<Segment> open_segs;



        // If there are no obstacles that are obstructing the view of the reference
        // Segment the entire Segment is open
        if (obstacle_segment_projections.size() == 0)
        {
            const Point center_of_segment =
                getPointsMean({segment.getSegStart(), segment.getEnd()});
            const Angle angle_of_entire_segment =
                ((segment.getSegStart() - origin)
                     .orientation()
                     .minDiff((segment.getEnd() - origin).orientation()))
                    .abs();

            return Shot(center_of_segment, angle_of_entire_segment);
        }

        // The first Angle is between the reference Segment and the first obstacle Segment
        // After this one, ever open angle is between segment(i).end and
        // segment(i+1).start
        open_segs.push_back(Segment(segment.getSegStart(),
                                    obstacle_segment_projections.front().getSegStart()));

        // The 'open' Segment in the space between consecutive 'blocking' Segments
        for (std::vector<Segment>::const_iterator it =
                 obstacle_segment_projections.begin();
             it != obstacle_segment_projections.end() - 1; it++)
        {
            open_segs.push_back(Segment(it->getEnd(), (it + 1)->getSegStart()));
        }

        // Lastly, the final open angle is between obstacles.end().getEnd() and
        // reference_segment.getEnd()
        open_segs.push_back(
            Segment(obstacle_segment_projections.back().getEnd(), segment.getEnd()));


        Segment largest_segment = *std::max_element(
            open_segs.begin(), open_segs.end(), [](const Segment &s1, const Segment &s2) {
                return s1.length() < s2.length();
            });

        const Point most_open_point =
            Point((largest_segment.getSegStart().x() + largest_segment.getEnd().x()) / 2,
                  (largest_segment.getSegStart().y() + largest_segment.getEnd().y()) / 2);
        const Angle largest_open_angle =
            ((largest_segment.getSegStart() - origin)
                 .orientation()
                 .minDiff((largest_segment.getEnd() - origin).orientation()))
                .abs();

        return Shot(most_open_point, largest_open_angle);
    }
}  // namespace Evaluation
