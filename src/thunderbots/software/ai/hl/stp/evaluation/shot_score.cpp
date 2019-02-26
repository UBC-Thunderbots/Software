
#include <shared/constants.h>
#include "shot_score.h"
#include "geom/circle.h"
#include "geom/util.h"


namespace {

    Evaluation::ShotDirAndScore getBestShotAtLine(Robot shooter, std::vector<Robot> all_robots, Point lower_point, Point upper_point) {

        std::vector<Point> obstacles;

        // Grab all robot positions as obstacles (Ignore the shooter)
        for( Robot robot : all_robots) {
            if( shooter.position() == robot.position() ) {
                continue;
            }
            else {
                obstacles.push_back(robot.position());
            }

        }
        // Calculate the direction of the best shot, and the 'score' represented by the angle of deviation from the direction that would still result in a goal.
        const std::pair<Vector, Angle> best_shot_data = angleSweepCircles(shooter.position(), lower_point, upper_point, obstacles, ROBOT_MAX_RADIUS_METERS);

        Evaluation::ShotDirAndScore shot_dir_and_score = { .shot_direction = best_shot_data.first, .score = best_shot_data.second.toRadians() };

        return shot_dir_and_score;
    }
}

namespace Evaluation {

    ShotDirAndScore getFriendlyRobotBestShot( Robot shooter, std::vector<Robot> all_robots, Field field ) {

        return getBestShotAtLine(shooter, all_robots, field.enemyGoalpostPos(), field.enemyGoalpostNeg());
    }

    ShotDirAndScore getEnemyRobotBestShot( Robot shooter, std::vector<Robot> all_robots, Field field ) {

        return getBestShotAtLine(shooter, all_robots, field.friendlyGoalpostPos(), field.friendlyGoalpostNeg());
    }
};