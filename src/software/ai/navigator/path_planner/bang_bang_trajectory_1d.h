#pragma once

#include <vector>

#include "software/ai/navigator/path_planner/itrajectory.h"

class BangBangTrajectory1D : public ITrajectory<double, double, double>
{
   public:
    struct TrajectoryPart  // TODO: Make private
    {
        Duration end_time;
        double position;  // TODO: Consider adding initial/final prefix
        double velocity;
        double acceleration;
    };

    BangBangTrajectory1D() = default;

    /**
     * Overwrites any existing trajectory with the newly generated trajectory
     *
     * @param initial_pos
     * @param final_pos
     * @param initial_vel
     * @param max_vel
     * @param max_accel
     * @param max_decel
     */
    void generate(double initial_pos, double final_pos, double initial_vel,
                  double max_vel, double max_accel, double max_decel);

    /**
     * Get position at time t
     *
     * @param t Duration elapsed since start of trajectory
     * @return position
     */
    double getPosition(Duration t) const override;

    /**
     * Get velocity at time t
     *
     * @param t Duration elapsed since start of trajectory
     * @return velocity
     */
    double getVelocity(Duration t) const override;

    /**
     * Get acceleration at time t
     *
     * @param t Duration elapsed since start of trajectory
     * @return acceleration
     */
    double getAcceleration(Duration t) const override;

    /**
     * Get total runtime of trajectory
     *
     * @return total time for trajectory in seconds
     */
    Duration getTotalTime() const override;

    // TODO: Added for testing purposes, remove later
    const std::vector<TrajectoryPart> &getTrajectoryParts() const;

   private:
    /**
     * Calculate the minimum distance required to stop (0 velocity) from a
     * given initial velocity.
     * @param initial_vel Initial velocity
     * @param max_decel Max achievable deceleration
     * @return Minimum distance required to stop
     */
    inline double closestPositionToStop(double initial_pos, double initial_vel,
                                        double max_decel) const;

    inline double triangularProfileStopPosition(double initial_pos, double initial_vel,
                                                double max_vel, double max_accel,
                                                double max_decel) const;

    /**
     *
     * @param initial_pos
     * @param final_pos
     * @param initial_vel
     * @param max_vel Assuming value is positive
     * @param max_accel Assuming value is positive
     * @param max_decel Assuming value is positive
     * @param time_offset
     */
    void generateTrapezoidalTrajectory(double initial_pos, double final_pos,
                                       double initial_vel, double max_vel,
                                       double max_accel, double max_decel,
                                       Duration time_offset = Duration::fromSeconds(0));

    void generateTriangularTrajectory(double initial_pos, double final_pos,
                                      double initial_vel, double max_accel,
                                      double max_decel,
                                      Duration time_offset = Duration::fromSeconds(0));

    size_t getTrajectoryIndexAtTime(Duration t) const;

    void getTrajPartAndDeltaTime(Duration t, TrajectoryPart &out_traj_part,
                                 Duration &out_t_delta) const;

    std::vector<TrajectoryPart> trajectory_parts;
};
