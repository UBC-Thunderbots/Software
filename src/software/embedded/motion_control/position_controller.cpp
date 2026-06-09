#include "software/embedded/motion_control/position_controller.h"

#include "proto/message_translation/tbots_protobuf.h"
#include "software/logger/logger.h"

Vector PositionController::step(const Point& position,
                                const TrajectoryPath& target_trajectory,
                                Duration elapsed_time, Duration delta_time)
{
    LOG(PLOTJUGGLER) << *createPlotJugglerValue(
        {{"target_position_x",
          target_trajectory.getPosition(elapsed_time.toSeconds()).x()},
         {"target_position_y",
          target_trajectory.getPosition(elapsed_time.toSeconds()).y()},
         {"actual_position_x", position.x()},
         {"actual_position_y", position.y()}});
    const Vector distance_from_destination =
        target_trajectory.getDestination() - position;

    if (distance_from_destination.length() < LINEAR_PURE_PID_THRESHOLD_METERS ||
        target_trajectory.getTotalTime() - elapsed_time.toSeconds() <
            PURE_PID_THRESHOLD_TIME)
    {
        // if target destination is close enough, use pure PID for velocity
        return Vector{
            x_pid_close_.step(distance_from_destination.x(), delta_time.toSeconds()),
            y_pid_close_.step(distance_from_destination.y(), delta_time.toSeconds())};
    }
    else
    {
        // feedforward trajectory velocity with small pid control effort
        const Vector error =
            target_trajectory.getPosition(elapsed_time.toSeconds()) - position;
        const Vector control_effort{x_pid_.step(error.x(), delta_time.toSeconds()),
                                    y_pid_.step(error.y(), delta_time.toSeconds())};
        return target_trajectory.getVelocity(elapsed_time.toSeconds()) + control_effort;
    }
}

void PositionController::reset()
{
    x_pid_.reset();
    y_pid_.reset();
    x_pid_close_.reset();
    y_pid_close_.reset();
}
