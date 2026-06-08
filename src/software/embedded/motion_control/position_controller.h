#pragma once

#include "software/ai/navigator/trajectory/trajectory_path.h"
#include "software/embedded/motion_control/controller.h"
#include "software/embedded/motion_control/pid_controller.h"
#include "software/geom/point.h"
#include "software/geom/vector.h"
#include "software/time/duration.h"

class PositionController : public MotionController<Point, TrajectoryPath, Vector>
{
   public:
    /**
     * Constructs a position controller that uses measurements over multiple time
     * intervals to calculate the target velocity to minimize error.
     */
    PositionController() = default;

    /**
     * Given a position and target trajectory, returns a target global velocity to
     * minimize error between the two.
     *
     * @param position The actual position.
     * @param target_trajectory The target 2D trajectory path.
     * @param elapsed_time The elapsed time since the trajectory was created.
     * @param delta_time The time passed since last time step.
     */
    Vector step(const Point& position, const TrajectoryPath& target_trajectory,
                Duration elapsed_time, Duration delta_time) override;

    /**
     * Resets the state of this position controller.
     */
    void reset() override;

   private:
    // TODO(#3737): tune constants
    PidController x_pid_{2.0, 0.0, 0.0, 0.0};
    PidController y_pid_{2.0, 0.0, 0.0, 0.0};

    PidController x_pid_close_{2.0, 1.0, 0.0, 5.0};
    PidController y_pid_close_{2.0, 1.0, 0.0, 5.0};

    static constexpr double LINEAR_PURE_PID_THRESHOLD_METERS = 0.3;
    static constexpr double PURE_PID_THRESHOLD_TIME = 0.25;
};
