#pragma once

#include "software/new_geom/point.h"
#include "software/new_geom/vector.h"
#include "software/time/timestamp.h"
#include "software/world/ball_state.h"

class BallStateWithTimestamp : public BallState
{
   public:
    /**
     * Creates a new ball state with timestamp information
     *
     * @param position The position of the ball, with coordinates in metres
     * @param velocity The velocity of the ball, in metres per second
     * @param timestamp The timestamp at which the ball was observed to be at the
     * given position and velocity
     */
    explicit BallStateWithTimestamp(const Point& position, const Vector& velocity, const Timestamp &timestamp);

    /**
     * Creates a new ball state with timestamp information
     *
     * @param ball_state The state of the ball
     * @param timestamp The timestamp at which the ball was observed to be
     * at the given state
     */
    explicit BallStateWithTimestamp(const BallState& ball_state, const Timestamp &timestamp);

    /**
     * Returns the timestamp of the ball represented by this state
     *
     * @return the timestamp of the ball represented by this state
     */
    Timestamp timestamp() const;

    /**
     * Returns the ball state, without any timestamp information
     *
     * @return the ball state without any timestamp information
     */
    BallState getBallState() const;

    /**
     * Defines the equality operator for a BallState. BallStates are equal if their
     * positions and velocities are the same
     *
     * @param other The ball state to compare against for equality
     * @return True if the other ball state is equal to this ball state, and false
     * otherwise
     */
    bool operator==(const BallStateWithTimestamp &other) const;

    /**
     * Defines the inequality operator for a BallState.
     *
     * @param other The ball state to compare against for inequality
     * @return True if the other ball state is not equal to this ball state, and false
     * otherwise
     */
    bool operator!=(const BallStateWithTimestamp &other) const;

   private:
    Timestamp timestamp_;
};
