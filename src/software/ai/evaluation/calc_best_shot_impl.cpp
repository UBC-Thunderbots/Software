#include "software/ai/evaluation/calc_best_shot_impl.h"

#include "software/geom/algorithms/acute_angle.h"
#include "software/geom/algorithms/multiple_segments.h"
#include "software/geom/algorithms/projection.h"

double calcShotOpenNetPercentage(const Field &field, const Point &shot_origin,
                                 const Shot &shot, TeamType goal)
{
    Angle goal_angle;
    if (goal == TeamType::FRIENDLY)
    {
        goal_angle = acuteAngle(field.friendlyGoalpostPos(), shot_origin,
                                field.friendlyGoalpostNeg())
                         .abs();
    }
    else
    {
        goal_angle =
            acuteAngle(field.enemyGoalpostPos(), shot_origin, field.enemyGoalpostNeg())
                .abs();
    }
    return shot.getOpenAngle().toDegrees() / goal_angle.toDegrees();
}

std::optional<Shot> calcMostOpenDirectionFromRobotObstacles(
    const Point &origin, const Segment &segment,
    const std::vector<Robot> &robot_obstacles)
{
    std::vector<Circle> obstacles;

    for (Robot robot : robot_obstacles)
    {
        obstacles.push_back(Circle(robot.position(), ROBOT_MAX_RADIUS_METERS));
    }

    return calcMostOpenDirectionFromCircleObstacles(origin, segment, obstacles);
}

std::optional<Shot> calcMostOpenDirectionFromCircleObstacles(
    const Point &origin, const Segment &segment, const std::vector<Circle> &obstacles)
{
    std::vector<Segment> obstacle_segment_projections;

    // If there are no obstacles, return the center of the Segment and the shot angle
    if (obstacles.size() == 0)
    {
        const Point center_of_segment = segment.midPoint();
        const Angle angle_of_entire_segment =
            ((segment.getStart() - origin)
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
        const Point center_of_segment = segment.midPoint();
        const Angle angle_of_entire_segment =
            ((segment.getStart() - origin)
                 .orientation()
                 .minDiff((segment.getEnd() - origin).orientation()))
                .abs();

        return std::make_optional(Shot(center_of_segment, angle_of_entire_segment));
    }
    std::vector<Segment> open_segs;

    open_segs = getEmptySpaceWithinParentSegment(obstacle_segment_projections, segment);

    Segment largest_segment;

    if (open_segs.size() >= 1)
    {
        largest_segment = *std::max_element(open_segs.begin(), open_segs.end(),
                                            [](const Segment &s1, const Segment &s2) {
                                                return s1.length() < s2.length();
                                            });
    }
    else
    {
        return std::nullopt;
    }


    const Point most_open_point =
        Point((largest_segment.getStart().x() + largest_segment.getEnd().x()) / 2,
              (largest_segment.getStart().y() + largest_segment.getEnd().y()) / 2);
    const Angle largest_open_angle =
        ((largest_segment.getStart() - origin)
             .orientation()
             .minDiff((largest_segment.getEnd() - origin).orientation()))
            .abs();

    return std::make_optional(Shot(most_open_point, largest_open_angle));
}
