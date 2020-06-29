#pragma once

#include <boost/circular_buffer.hpp>
#include <map>

#include "software/new_geom/point.h"
#include "software/new_geom/vector.h"
#include "software/time/timestamp.h"
#include "software/world/ball_state.h"

/**
 * Represents the state of a ball at a specific time
 */
class TimestampedBallState
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
    explicit TimestampedBallState(const Point &position, const Vector &velocity,
                                  const Timestamp &timestamp);

    /**
     * Creates a new ball state with timestamp information
     *
     * @param ball_state The state of the ball
     * @param timestamp The timestamp at which the ball was observed to be
     * at the given state
     */
    explicit TimestampedBallState(const BallState &ball_state,
                                  const Timestamp &timestamp);

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
    BallState state() const;

    /**
     * Defines the equality operator for a TimestampedBallState. TimestampedBallStates
     * are equal if their positions and velocities are the same
     *
     * @param other The ball state to compare against for equality
     * @return True if the other ball state is equal to this ball state, and false
     * otherwise
     */
    bool operator==(const TimestampedBallState &other) const;

    /**
     * Defines the inequality operator for a TimestampedBallState.
     *
     * @param other The ball state to compare against for inequality
     * @return True if the other ball state is not equal to this ball state, and false
     * otherwise
     */
    bool operator!=(const TimestampedBallState &other) const;

   private:
    BallState ball_state_;
    Timestamp timestamp_;
};

using BallHistory = boost::circular_buffer<TimestampedBallState>;
