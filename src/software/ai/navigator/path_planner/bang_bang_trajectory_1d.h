#pragma once

#include <vector>

#include "software/ai/navigator/path_planner/itrajectory.h"

class BangBangTrajectory1D : public ITrajectory<double, double, double>
{
   public:
    /**
     * A complete trajectory is made out of multiple TrajectoryParts.
     * Each trajectory part has a starting position, velocity, and acceleration
     * and an end time relative to the start of the trajectory so we know when
     * to switch to the next trajectory part.
     */
    struct TrajectoryPart
    {
        Duration end_time;
        double position;
        double velocity;
        double acceleration;
    };

    BangBangTrajectory1D() = default;

    /**
     * Generate a 1D trajectory from the initial position to the final position with the
     * given initial velocity and kinematic constraints.
     * @note This method will overwrite the existing trajectory.
     * @note The generated trajectory will always have a final velocity of 0
     *
     * @param initial_pos Starting position of the trajectory
     * @param final_pos Destination. Where the trajectory should end at
     * @param initial_vel The velocity at the start of the trajectory
     * @param max_vel The maximum velocity (magnitude) the trajectory could have
     * @param max_accel The maximum acceleration the trajectory could have
     * @param max_decel The maximum deceleration the trajectory could have
     */
    void generate(double initial_pos, double final_pos, double initial_vel,
                  double max_vel, double max_accel, double max_decel);

    /**
     * Get position at time t
     *
     * @param t Duration elapsed since start of trajectory
     * @return The position at time t
     */
    double getPosition(Duration t) const override;

    /**
     * Get velocity at time t
     *
     * @param t Duration elapsed since start of trajectory
     * @return The velocity at time t
     */
    double getVelocity(Duration t) const override;

    /**
     * Get acceleration at time t
     *
     * @param t Duration elapsed since start of trajectory
     * @return The acceleration at time t
     */
    double getAcceleration(Duration t) const override;

    /**
     * Get total runtime of trajectory
     *
     * @return total time for trajectory in seconds
     */
    Duration getTotalTime() const override;

    /**
     * Get the trajectory parts that make up the generated trajectory
     *
     * @return Trajectory parts
     */
    const std::vector<TrajectoryPart> &getTrajectoryParts() const;

   private:
    /**
     * Generate a trapezoidal trajectory and fill in `trajectory_parts`.
     * Trapezoidal trajectory gets its name from the shape of the generated velocity vs
     * time graph. The velocity will accelerate to max velocity, cruise at max velocity,
     * then decelerate to 0
     * @assumes that there is enough distance to accelerate from initial velocity to max
     * velocity, and decelerate from max velocity to 0 velocity.
     *
     * @param initial_pos Starting position of the trajectory
     * @param final_pos Destination. Where the trajectory should end at
     * @param initial_vel The velocity at the start of the trajectory
     * @param max_vel The maximum velocity (magnitude) the trajectory could have. Assumes
     * value is positive
     * @param max_accel The maximum acceleration the trajectory could have. Assumes value
     * is positive
     * @param max_decel The maximum deceleration the trajectory could have. Assumes value
     * is positive
     * @param time_offset The time offset to start the trajectory at
     */
    void generateTrapezoidalTrajectory(double initial_pos, double final_pos,
                                       double initial_vel, double max_vel,
                                       double max_accel, double max_decel,
                                       Duration time_offset = Duration::fromSeconds(0));

    /**
     * Generate a triangular trajectory and fill in `trajectory_parts`.
     * Triangular trajectory has the velocity vs time graph look like a triangle. The
     * velocity will accelerate to a value less than or equal to the max velocity, then
     * decelerate to 0.
     * @assumes that there is not enough time for the trajectory to cruise at max
     * velocity, given the distance that needs to be travelled.
     *
     * @param initial_pos Starting position of the trajectory
     * @param final_pos Destination. Where the trajectory should end at
     * @param initial_vel The velocity at the start of the trajectory
     * @param max_accel The maximum acceleration the trajectory could have. Assumes value
     * is positive
     * @param max_decel The maximum deceleration the trajectory could have. Assumes value
     * is positive
     * @param time_offset The time offset to start the trajectory at
     */
    void generateTriangularTrajectory(double initial_pos, double final_pos,
                                      double initial_vel, double max_accel,
                                      double max_decel,
                                      Duration time_offset = Duration::fromSeconds(0));

    /**
     * Calculates the closest position at which the trajectory could stop at (velocity =
     * 0) given the initial velocity.
     *
     * @param initial_vel Initial velocity
     * @param max_decel Max achievable deceleration
     * @return Closest position to the initial position that it can stop at
     */
    inline double closestPositionToStop(double initial_pos, double initial_vel,
                                        double max_decel) const;

    /**
     * Calculates the position at which the robot will be at if it accelerates from the
     * initial velocity to the max velocity, then decelerates to 0 velocity.
     *
     * @param initial_pos Starting position of the trajectory
     * @param initial_vel The velocity at the start of the trajectory
     * @param max_vel The maximum velocity (magnitude) the trajectory could have. Assumes
     * value is positive
     * @param max_accel The maximum acceleration the trajectory could have. Assumes value
     * is positive
     * @param max_decel The maximum deceleration the trajectory could have. Assumes value
     * is positive
     * @param direction Direction of the trajectory. 1 for moving in the positive direction,
     * -1 for moving in the negative direction
     * @return The final position of the robot after a triangular profile trajectory to
     * the max velocity
     */
    inline double triangularProfileStopPosition(double initial_pos, double initial_vel, double max_vel,
                                                double max_accel, double max_decel, double direction) const;

    /**
     * Get the index of `trajectory_parts` that the robot is at at time t
     *
     * @param t Duration elapsed since start of trajectory
     * @return Index of `trajectory_parts` that the robot is at at time t
     */
    size_t getTrajectoryIndexAtTime(Duration t) const;

    /**
     * Helper for getting the trajectory part at time t, and the time delta
     * between the start of the trajectory part and time t.
     *
     * @param t Duration elapsed since start of trajectory
     * @param out_traj_part Out parameter for the trajectory part at time t
     * @param out_t_delta Out parameter for the time delta between the start of the
     * trajectory part and time t
     */
    void getTrajPartAndDeltaTime(Duration t, TrajectoryPart &out_traj_part,
                                 Duration &out_t_delta) const;

    std::vector<TrajectoryPart> trajectory_parts;
};
