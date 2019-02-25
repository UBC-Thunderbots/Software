
#include <shared/constants.h>
#include "shot_score.h"
#include "geom/circle.h"

namespace Evaluation {

    ShotAndScore get_best_shot( Robot shooter, std::vector<Robot> all_robots, Field field, bool shooter_is_friendly) {

        // remove shooter from friendly team and combine friendly/enemy
        // model each robot as a 'circle' object and gather tangents of each circle to center of player
        // Now have lots of line segments, see which ones have a intersect with the enemy net
        // See which pair of adjacent lines have the greatest view of the net
        std::vector<Circle> obstacles;

        // Set the goal posts to reflect where the shooter would be shooting
        if(shooter_is_friendly) {
            Point goal_post_pos = field.enemyGoalpostPos();
            Point goal_post_neg = field.enemyGoalpostNeg();
        }
        else {
            Point goal_post_pos = field.friendlyGoalpostPos();
            Point goal_post_neg = field.friendlyGoalpostNeg();
        }

        // Create a circle object for all robots except the shooter bot
        for( Robot robot : all_robots ) {
            if( (shooter.position() - robot.position()).len() != 0 ) {
                obstacles.push_back( Circle(robot.position(), ROBOT_MAX_RADIUS_METERS));
            }
        }



};