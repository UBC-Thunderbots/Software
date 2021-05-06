#include "software/simulated_tests/terminating_validation_functions/robot_blocking_shot.h"

#include "software/logger/logger.h"


void robotBlockingShot(RobotId robot_id, std::shared_ptr<World> world_ptr,
                       ValidationCoroutine::push_type& yield)
{
    auto robot_blocking_shot = [robot_id](std::shared_ptr<World> world_ptr) {
        std::optional<Robot> robot_optional =
            world_ptr->friendlyTeam().getRobotById(robot_id);
        if (!robot_optional.has_value())
        {
            LOG(FATAL) << "There is no robot with ID: " + std::to_string(robot_id);
        }
        Point ball_position  = world_ptr->ball().position();
        Vector post_pos      = world_ptr->field().friendlyGoalpostPos() - ball_position;
        Vector post_neg      = world_ptr->field().friendlyGoalpostNeg() - ball_position;
        Vector ball_to_robot = robot_optional.value().position() - ball_position;

        // We only need the sign of the cross product
        double cross = post_pos.cross(post_neg);


        // If post_pos X ball_to_robot is the same sign as the cross and ball_to_robot is
        // the same sign as the cross, this means ball_to_robot must be in between
        // post_post and post_neg. If we multiply two doubles and they are greater than 0,
        // then they are the same sign. They will only equal 0 if they are parallel
        // vectors e.g. the robot is blocking the goal post.
        return post_pos.cross(ball_to_robot) * cross >= 0 &&
               ball_to_robot.cross(post_neg) * cross >= 0;
    };


    while (!robot_blocking_shot(world_ptr))
    {
        yield("Robot with ID " + std::to_string(robot_id) + " is not blocking the shot");
    }
}
