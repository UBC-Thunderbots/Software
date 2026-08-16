#pragma once

#include "software/ai/navigator/trajectory/trajectory_path.h"
#include "software/embedded/motion_control/controller.h"
#include "software/embedded/motion_control/pid_controller.h"
#include "software/geom/point.h"
#include "software/geom/vector.h"

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
     * @param elapsed_time_s The elapsed time since the trajectory was created in seconds.
     * @param delta_time_s The time passed since last time step in seconds.
     */
    Vector step(const Point& position, const TrajectoryPath& target_trajectory,
                double elapsed_time_s, double delta_time_s) override;

    /**
     * Resets the state of this position controller.
     */
    void reset() override;

   private:
    // TODO(#3737): tune constants
    PidController<double> x_pid_{0.2, 0.0, 0.0, 0.0};
    PidController<double> y_pid_{0.2, 0.0, 0.0, 0.0};

    static constexpr double MAX_DAMPENING_VELOCITY_DISTANCE_M = 0.05;
};
