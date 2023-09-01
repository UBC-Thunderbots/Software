#pragma once

#include "software/time/duration.h"

/**
 * A trajectory is a path that is parameterized by time. It is a function that
 * takes in a time and returns a position, velocity, and acceleration.
 *
 * @tparam P The type of position
 * @tparam V The type of velocity
 * @tparam A The type of acceleration
 */
template <class P, class V, class A>
class ITrajectory
{
   public:
    virtual ~ITrajectory() = default;

    /**
     * Get position at time t
     *
     * @param t Duration elapsed since start of trajectory
     * @return position
     */
    virtual P getPosition(Duration t) const = 0;


    /**
     * Get velocity at time t
     *
     * @param t Duration elapsed since start of trajectory
     * @return velocity
     */
    virtual V getVelocity(Duration t) const = 0;


    /**
     * Get acceleration at time t
     *
     * @param t Duration elapsed since start of trajectory
     * @return acceleration
     */
    virtual A getAcceleration(Duration t) const = 0;


    /**
     * Get total runtime of trajectory
     *
     * @return total time for trajectory
     */
    virtual Duration getTotalTime() const = 0;


    /**
     * Get the final desired destination
     * @return The position which the trajectory ends at
     */
    P getDestination() const
    {
        return getPosition(getTotalTime());
    }
};
