#pragma once

#include "software/ai/navigator/path_planner/ITrajectory.h"

class BangBangTrajectory1D : public ITrajectory<double, double, double>
{
    struct TrajectoryPart
    {
        Duration end_time;
        double position; // TODO: Consider adding initial/final prefix
        double velocity;
        double acceleration;
    };

public:
    /**
     * Constructor
     */
    BangBangTrajectory1D(double initial_pos, double final_pos, double initial_vel, double final_vel,
                         double max_vel,
                         double max_accel, double max_decel);

    /**
     * Get position at time t
     *
     * @param t Duration elapsed since start of trajectory
     * @return position
     */
    double getPosition(const Duration t) override;

    /**
     * Get velocity at time t
     *
     * @param t Duration elapsed since start of trajectory
     * @return velocity
     */
    double getVelocity(const Duration t) override;

    /**
     * Get acceleration at time t
     *
     * @param t Duration elapsed since start of trajectory
     * @return acceleration
     */
    double getAcceleration(const Duration t) override;

    /**
     * Get total runtime of trajectory
     *
     * @return total time for trajectory in seconds
     */
    Duration getTotalTime() override;

private:
    inline double closestPositionToReachVelocity(double initial_position, double initial_velocity,
                                                 double max_deceleration);
    std::vector<TrajectoryPart> trajectory_parts;
};