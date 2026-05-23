#pragma once

#include "software/ai/navigator/trajectory/trajectory_path.h"
#include "software/embedded/motion_control/pid_controller.h"
#include "software/geom/point.h"
#include "software/geom/vector.h"
#include "software/time/duration.h"

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
     * Given an error, returns a target global velocity to minimize it.
     *
     * @param position The actual position
     * @param target_path The target trajectory path
     * @param delta_time The time passed since last time step.
     */
    Vector step(const Point& position, const TrajectoryPath& target_path,
                Duration time_since_trajectory_creation, double delta_time = 1.0);

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

    static constexpr double LINEAR_PURE_PID_THRESHOLD_METERS = 0.5;
};
