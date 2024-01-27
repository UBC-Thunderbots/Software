#pragma once

/**
 * Interface for a trajectory
 *
 * A trajectory is a path that is parameterized by time. It is a function that
 * takes in a time and returns a position, velocity, and acceleration.
 *
 * @tparam P The type of position
 * @tparam V The type of velocity
 * @tparam A The type of acceleration
 */
template <class P, class V, class A>
class Trajectory
{
   public:
    virtual ~Trajectory() = default;

    /**
     * Get position at time t
     *
     * @param t_sec Duration elapsed since start of trajectory in seconds
     * @return position
     */
    virtual P getPosition(double t_sec) const = 0;


    /**
     * Get velocity at time t
     *
     * @param t_sec Duration elapsed since start of trajectory in seconds
     * @return velocity
     */
    virtual V getVelocity(double t_sec) const = 0;


    /**
     * Get acceleration at time t
     *
     * @param t_sec Duration elapsed since start of trajectory in seconds
     * @return acceleration
     */
    virtual A getAcceleration(double t_sec) const = 0;


    /**
     * Get total runtime of trajectory
     *
     * @return total time for trajectory in seconds
     */
    virtual double getTotalTime() const = 0;


    /**
     * Get the final desired destination
     * @return The position which the trajectory ends at
     */
    P getDestination() const
    {
        return getPosition(getTotalTime());
    }
};
