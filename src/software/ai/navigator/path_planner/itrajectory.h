#pragma once

#include "software/time/duration.h"

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
     * @return total time for trajectory in seconds
     */
    virtual Duration getTotalTime() const = 0;


    /**
     * Get the final desired destination
     * @return the next destination, if this trajectory is divided into multiple
     * subtract-paths
     */
    P getDestination() const
    {
        return getPosition(getTotalTime());
    }
};
