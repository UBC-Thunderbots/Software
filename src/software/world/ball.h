#pragma once

#include <boost/circular_buffer.hpp>
#include <optional>

#include "software/new_geom/point.h"
#include "software/new_geom/vector.h"
#include "software/time/timestamp.h"
#include "software/world/timestamped_ball_state.h"

class Ball final
{
   public:
    Ball() = delete;

    /**
     * Creates a new ball with the given initial state
     *
     * @param position The position of the ball, with coordinates in metres
     * @param velocity The velocity of the ball, in metres per second
     * @param timestamp The timestamp at which the ball was observed to be at the
     * given position and velocity
     * @param history_size The number of previous ball states that should be stored. Must
     * be > 0
     */
    explicit Ball(const Point &position, const Vector &velocity,
                  const Timestamp &timestamp, unsigned int history_size = 20);

    /**
     * Creates a new ball with the given initial state
     *
     * @param initial_state The initial state of the ball
     * @param history_size The number of previous ball states that should be stored. Must
     * be > 0
     */
    explicit Ball(const TimestampedBallState &initial_state,
                  unsigned int history_size = 20);

    /**
     * Returns the current state of the ball
     */
    TimestampedBallState currentState() const;

    /**
     * Updates the ball with new data, updating the current data as well as the predictive
     * model
     *
     * @param new_state the new state of the ball
     */
    void updateState(const TimestampedBallState &new_state);

    /**
     * Updates the ball's state to be its predicted state at the given timestamp.
     * The timestamp must be >= the ball's last update timestamp
     *
     * @throws std::invalid_argument if the ball is updated with a time from the past
     * @param timestamp The timestamp at which to update the ball's state to. Must
     * be >= the ball's last update timestamp
     */
    void updateStateToPredictedState(const Timestamp &timestamp);

    /**
     * Returns the timestamp for when this ball's data was last updated
     *
     * @return the timestamp for when this ball's data was last updated
     */
    Timestamp lastUpdateTimestamp() const;

    /**
     * Returns the current position of the ball
     *
     * @return the current position of the ball
     */
    Point position() const;

    /**
     * Returns the estimated position of the ball at a future time, relative to when the
     * ball was last updated.
     *
     * @param duration_in_future The relative amount of time in the future
     * at which to predict the ball's position. Value must be >= 0.
     * For example, a value of 1.5 seconds would return the estimated position of the ball
     * 1.5 seconds in the future.
     *
     * @throws std::invalid_argument if the ball is estimating the position with a time
     * from the past
     * @return the estimated position of the ball at the given number of milliseconds
     * in the future. Coordinates are in metres.
     */
    Point estimatePositionAtFutureTime(const Duration &duration_in_future) const;

    /**
     * Returns the current velocity of the ball
     *
     * @return the current velocity of the ball
     */
    Vector velocity() const;

    /**
     * Returns the estimated velocity of the ball at a future time, relative to when the
     * ball was last updated
     *
     * @param duration_in_future The relative amount of time in the future
     * at which to predict the ball's velocity. Value must be >= 0.
     * For example, a value of 1.5 seconds would return the estimated velocity of the ball
     * 1.5 seconds in the future.
     *
     * @throws std::invalid_argument if the ball is estimating the velocity with a time
     * from the past
     * @return the estimated velocity of the ball at the given number of milliseconds
     * in the future. Coordinates are in metres.
     */
    Vector estimateVelocityAtFutureTime(const Duration &duration_in_future) const;

    /**
     * Gets the previous states stored in states_
     *
     * @return The circular buffer containing the state history starting with the newest
     * available data at index 0
     */
    boost::circular_buffer<TimestampedBallState> getPreviousStates() const;

    /**
     * Finds an update timestamp that is close to the provided timestamp and returns the
     * index of the timestamp in the history buffer.
     *
     * @param timestamp timestamp of the update state index we wish to fetch
     * @return Index of the ball's update timestamp closest to the desired time or a
     * std::nullopt if there is not matching timestamp.
     */
    std::optional<int> getHistoryIndexFromTimestamp(const Timestamp &timestamp) const;

    /**
     * Defines the equality operator for a Ball. Balls are equal if their positions and
     * velocities are the same
     *
     * @param other The Ball to compare against for equality
     * @return True if the other ball is equal to this ball, and false otherwise
     */
    bool operator==(const Ball &other) const;

    /**
     * Defines the inequality operator for a Ball.
     *
     * @param other The ball to compare against for inequality
     * @return True if the other ball is not equal to this ball, and false otherwise
     */
    bool operator!=(const Ball &other) const;

   private:
    // All previous states of the ball, with the most recent state at the front of the
    // queue, This buffer will never be empty as it's initialized with a BallState on
    // creation
    // The buffer size (history_size) must be > 0
    boost::circular_buffer<TimestampedBallState> states_;
};
