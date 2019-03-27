
#include "shoot.h"

#include <shared/constants.h>

#include "geom/circle.h"
#include "geom/util.h"


namespace
{
    Evaluation::ShotDirAndScore getBestShotAtSegment(Robot shooter,
                                                     std::vector<Point> all_obstacles,
                                                     Segment segment)
    {
        std::vector<Point> obstacles;

        // Make the list of obstacles does not include the shooter
        for (Point position : all_obstacles)
        {
            if (shooter.position() == position)
            {
                continue;
            }
            else
            {
                obstacles.push_back(position);
            }
        }
        // Calculate the direction of the best shot, and the 'score' represented by the
        // angle of deviation from the direction that would still result in a goal.
        const std::pair<Vector, Angle> best_shot_data =
            angleSweepCircles(shooter.position(), segment.getSegStart(), segment.getEnd(),
                              obstacles, ROBOT_MAX_RADIUS_METERS);

        Evaluation::ShotDirAndScore shot_dir_and_score;
        shot_dir_and_score.shot_direction = best_shot_data.first;
        shot_dir_and_score.score          = best_shot_data.second.toRadians();

        return shot_dir_and_score;
    }
}  // namespace

namespace Evaluation
{
    ShotDirAndScore getFriendlyRobotBestShotGoal(Robot shooter,
                                                 std::vector<Point> all_obstacles,
                                                 Field field)
    {
        return getBestShotAtSegment(
            shooter, all_obstacles,
            Segment(field.enemyGoalpostPos(), field.enemyGoalpostNeg()));
    }

    ShotDirAndScore getEnemyRobotBestShotGoal(Robot shooter,
                                              std::vector<Point> all_obstacles,
                                              Field field)
    {
        return getBestShotAtSegment(
            shooter, all_obstacles,
            Segment(field.friendlyGoalpostPos(), field.friendlyGoalpostNeg()));
    }
};  // namespace Evaluation