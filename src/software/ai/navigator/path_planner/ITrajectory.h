#pragma once

#include "software/time/duration.h"

class ITrajectory<P, V, A>
{
   public:
    /**
     * Get position at time t
     *
     * @param t Duration elapsed since start of trajectory
     * @return position
     */
    virtual P getPosition(const Duration t) = 0;


    /**
     * Get velocity at time t
     *
     * @param t Duration elapsed since start of trajectory
     * @return velocity
     */
    virtual V getVelocity(const Duration t) = 0;


    /**
     * Get acceleration at time t
     *
     * @param t Duration elapsed since start of trajectory
     * @return acceleration
     */
    virtual A getAcceleration(const Duration t) = 0;


    /**
     * Get total runtime of trajectory
     *
     * @return total time for trajectory in seconds
     */
    virtual Duration getTotalTime() = 0;


    /**
     * Get the final desired destination
     * @param t Duration elapsed since start of trajectory
     * @return the next destination, if this trajectory is divided into multiple subtract-pathes
     */
    P getDestination(const Duration t)
    {
        return getPosition(getTotalTime());
    }
};