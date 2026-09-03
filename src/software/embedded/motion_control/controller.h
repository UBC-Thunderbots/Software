#pragma once

#include "software/time/duration.h"

/**
 * A generic interface for a motion controller.
 *
 * @tparam StateType The type of the system's current feedback state
 * @tparam TrajectoryType The type of the state's trajectory
 * @tparam OutputType The type of the calculated control effort
 */
template <typename StateType, typename TrajectoryType, typename OutputType>
class MotionController
{
   public:
    /**
     * Evaluates a single time step of the control loop and returns an output to minimize
     * error between the current state and target trajectory.
     *
     * @param current_state The actual measured current state.
     * @param target_trajectory The target trajectory of motion.
     * @param elapsed_time The total elapsed time since the trajectory was created.
     * @param delta_time The elapsed time since the last time step.
     */
    virtual OutputType step(const StateType& current_state,
                            const TrajectoryType& target_trajectory,
                            Duration elapsed_time, Duration delta_time) = 0;

    /**
     * Resets the internal state of the motion controller. For example, any
     * accumulated error terms used for calculating control effort.
     */
    virtual void reset() = 0;
};
