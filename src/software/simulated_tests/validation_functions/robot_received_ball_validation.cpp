#include "software/simulated_tests/validation_functions/robot_received_ball_validation.h"
#include "software/ai/evaluation/robot.h"

void robotReceivedBall(Robot robot, std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield)
{
    auto robot_received_ball = [robot](std::shared_ptr<World> world_ptr) {
        Vector vector_to_test_point = world_ptr->ball().position() - robot.position();
        static const double POSSESSION_THRESHOLD_METERS = ROBOT_MAX_RADIUS_METERS + 0.2;
        if (vector_to_test_point.length() > POSSESSION_THRESHOLD_METERS)
        {
            return false;
        }
        else
        {
            // check that ball is in a 90-degree cone in front of the robot
            auto ball_to_robot_angle =
                    robot.orientation().minDiff(vector_to_test_point.orientation());
            return (ball_to_robot_angle < Angle::fromDegrees(45.0));
        }
    };

    while (!robot_received_ball(world_ptr))
    {
        yield();
    }
}
