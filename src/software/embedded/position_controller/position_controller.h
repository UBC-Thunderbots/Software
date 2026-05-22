#pragma once

#include "software/embedded/position_controller/pid_controller.h"
#include "software/geom/vector.h"

// TODO: create angular velocity contorller as well

class PositionController
{
   public:
    /**
     * Constructs a position controller that uses error over multiple time
     * increments to calculate control effort to minimize it.
     */
    PositionController() = default;

    /**
     * Given an error, returns the control effort to minimize it.
     *
     * @param error The amount of error between desired and actual output
     * in both x and y coordinate.
     * @param delta_time The time passed since last time step.
     */
    Vector step(const Vector& error, double delta_time = 1.0);

    /**
     * Resets the state of this position controller.
     */
    void reset();

   private:
    // TODO: tune constants
    PidController x_pid_{0.8, 0.0, 0.0, 0.0};
    PidController y_pid_{0.8, 0.0, 0.0, 0.0};

    PidController x_pid_close_{2.0, 0.0, 0.0, 0.0};
    PidController y_pid_close_{2.0, 0.0, 0.0, 0.0};

    static constexpr double LINEAR_PURE_PID_THRESHOLD_METERS   = 0.5;
    static constexpr double ANGULAR_PURE_PID_THRESHOLD_DEGREES = 25;
};
