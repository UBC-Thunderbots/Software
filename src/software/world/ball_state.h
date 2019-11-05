#pragma once

#include <vector>

#include "software/new_geom/point.h"
#include "software/util/time/timestamp.h"

class BallState final
{
   public:
    /**
     * Creates a new ball state with the given position, velocity, and timestamp
     *
     * @param position The position of the ball, with coordinates in metres
     * @param velocity The velocity of the ball, in metres per second
     * @param timestamp The timestamp at which the ball was observed to be at the
     * given position and velocity
     */
    explicit BallState(Point position, Vector velocity, const Timestamp &timestamp);

    /**
     * Returns the position of the ball represented by this state
     *
     * @return the position of the ball represented by this state
     */
    Point position() const;

    /**
     * Returns the velocity of the ball represented by this state
     *
     * @return the velocity of the ball represented by this state
     */
    Vector velocity() const;

    /**
     * Returns the timestamp of the ball represented by this state
     *
     * @return the timestamp of the ball represented by this state
     */
    Timestamp timestamp() const;

    /**
     * Defines the equality operator for a BallState. BallStates are equal if their
     * positions and velocities are the same
     *
     * @param other The ball state to compare against for equality
     * @return True if the other ball state is equal to this ball state, and false
     * otherwise
     */
    bool operator==(const BallState &other) const;

    /**
     * Defines the inequality operator for a BallState.
     *
     * @param other The ball state to compare against for inequality
     * @return True if the other ball state is not equal to this ball state, and false
     * otherwise
     */
    bool operator!=(const BallState &other) const;

   private:
    Point position_;

    Vector velocity_;

    Timestamp timestamp_;
};
