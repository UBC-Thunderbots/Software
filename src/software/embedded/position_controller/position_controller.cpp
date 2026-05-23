#include "software/embedded/position_controller/position_controller.h"

Vector PositionController::step(const Point& position, const TrajectoryPath& target_path,
                                Duration time_since_trajectory_creation,
                                double delta_time)
{
    const Vector distance_from_destination = target_path.getDestination() - position;

    if (distance_from_destination.lengthSquared() < LINEAR_PURE_PID_THRESHOLD_METERS)
    {
        // if target destination is close enough, use pure PID for velocity
        return Vector{x_pid_close_.step(distance_from_destination.x(), delta_time),
                      y_pid_close_.step(distance_from_destination.y(), delta_time)};
    }
    else
    {
        // feedforward trajectory velocity with small pid control effort
        const Vector error =
            target_path.getPosition(time_since_trajectory_creation.toSeconds()) -
            position;
        const Vector control_effort{x_pid_.step(error.x(), delta_time),
                                    y_pid_.step(error.y(), delta_time)};
        return target_path.getVelocity(time_since_trajectory_creation.toSeconds()) +
               control_effort;
    }
}

void PositionController::reset()
{
    x_pid_.reset();
    y_pid_.reset();
    x_pid_close_.reset();
    y_pid_close_.reset();
}
