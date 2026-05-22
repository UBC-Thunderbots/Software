#include "software/embedded/position_controller/position_controller.h"

Vector PositionController::step(const Vector& error, double delta_time)
{
    // if close enough, use special PID to destination
    if (error.lengthSquared() < LINEAR_PURE_PID_THRESHOLD_METERS)
    {
        return Vector{x_pid_close_.step(error.x(), delta_time),
                      y_pid_close_.step(error.y(), delta_time)};
    }
    else
    {
        return Vector{x_pid_.step(error.x(), delta_time),
                      y_pid_.step(error.y(), delta_time)};
    }
}

void PositionController::reset()
{
    x_pid_.reset();
    y_pid_.reset();
    x_pid_close_.reset();
    y_pid_close_.reset();
}
