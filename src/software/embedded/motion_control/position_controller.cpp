#include "software/embedded/motion_control/position_controller.h"

#include "software/geom/algorithms/distance.h"

Vector PositionController::step(const Point& position,
                                const TrajectoryPath& target_trajectory,
                                Duration elapsed_time, Duration delta_time)
{
    // feedforward trajectory velocity with small pid control effort
    const Vector error =
        target_trajectory.getPosition(elapsed_time.toSeconds()) - position;
    const Vector control_effort{x_pid_.step(error.x(), delta_time.toSeconds()),
                                y_pid_.step(error.y(), delta_time.toSeconds())};
    double distance_to_destination =
        distance(position, target_trajectory.getDestination());

    Vector local_velocity =
        target_trajectory.getVelocity(elapsed_time.toSeconds()) + control_effort;

    // Dampen velocity as we get closer to the destination to reduce jittering
    if (distance_to_destination < MAX_DAMPENING_VELOCITY_DISTANCE_M)
    {
        local_velocity *= distance_to_destination / MAX_DAMPENING_VELOCITY_DISTANCE_M;
    }
    return local_velocity;
}

void PositionController::reset()
{
    x_pid_.reset();
    y_pid_.reset();
}
