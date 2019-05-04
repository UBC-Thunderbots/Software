#pragma once

#include <boost/circular_buffer.hpp>
#include <vector>

#include "geom/point.h"
#include "util/time/timestamp.h"

class Ball final
{
   public:
    /**
     * Creates a new ball with the given position and velocity
     *
     * @param position The position of the ball, with coordinates in metres
     * @param velocity The velocity of the ball, in metres per second
     * @param timestamp The timestamp at which the ball was observed to be at the
     * given position and velocity
     * @param history_duration The number of previous ball states that should be stored.
     *
     */
    explicit Ball(Point position, Vector velocity, const Timestamp& timestamp,
                  unsigned int history_duration = 20);

    /**
     * Updates the ball with new data, updating the current data as well as the predictive
     * model
     *
     * @param new_position the new position of the ball, defined in metres
     * @param new_velocity the new velocity of the ball, defined in metres per second
     * @param timestamp The timestamp at which the ball was observed to be at the given
     * position and velocity. The timestamp must be >= the ball's latest update timestamp
     */
    void updateState(const Point& new_position, const Vector& new_velocity,
                     const Timestamp& timestamp);

    /**
     * Updates the ball with new data, updating the current data as well as the predictive
     * model
     *
     * @throws std::invalid_argument if the ball is updated with a time from the past
     * @param new_ball_data A ball containing new ball data
     */
    void updateState(const Ball& new_ball_data);

    /**
     * Updates the ball's state to be its predicted state at the given timestamp.
     * The timestamp must be >= the ball's last update timestamp
     *
     * @throws std::invalid_argument if the ball is updated with a time from the past
     * @param timestamp The timestamp at which to update the ball's state to. Must
     * be >= the ball's last update timestamp
     */
    void updateStateToPredictedState(const Timestamp& timestamp);

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
    Point estimatePositionAtFutureTime(const Duration& duration_in_future) const;

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
    Vector estimateVelocityAtFutureTime(const Duration& duration_in_future) const;

    /**
     * Gets the buffer which holds all the previous position states of the ball
     *
     * @return Vector containing the position history starting with the oldest available
     * data at index 0
     */
    std::vector<Point> getPreviousPositions();

    /**
     * Gets the buffer which holds all the previous velocity states of the ball
     *
     * @return Vector containing the velocity history starting with the oldest available
     * data at index 0
     */
    std::vector<Vector> getPreviousVelocities();

    /**
     * Gets the buffer which holds all the timestamps of the previous states
     *
     * @return Vector containing the update timestamp history starting with the oldest
     * available data at index 0
     */
    std::vector<Timestamp> getPreviousTimestamps();

    /**
     * Adds a state to the front of the circular buffers storing the state of histories of
     * the ball
     *
     * @param position Position of ball
     * @param velocity Velocity of ball
     * @param timestamp Time that ball   was in this state
     */
    void addStateToBallHistory(const Point& position, const Vector& velocity,
                               const Timestamp& timestamp);

    /**
     * Defines the equality operator for a Ball. Balls are equal if their positions and
     * velocities are the same
     *
     * @param other The Ball to compare against for equality
     * @return True if the other ball is equal to this ball, and false otherwise
     */
    bool operator==(const Ball& other) const;

    /**
     * Defines the inequality operator for a Ball.
     *
     * @param other The ball to compare against for inequality
     * @return True if the other ball is not equal to this ball, and false otherwise
     */
    bool operator!=(const Ball& other) const;

   private:
    // All previous positions of the ball, with the most recent position at the front of
    // the queue, coordinates in meters
    boost::circular_buffer<Point> positions_;
    // All previous velocities of the ball, with the most recent velocity at the front of
    // the queue, coordinates in meters
    boost::circular_buffer<Vector> velocities_;
    // All previous timestamps of when the ball was updated , with the most recent
    // timestamp at the front of the queue, coordinates in meters
    boost::circular_buffer<Timestamp> last_update_timestamps;
};
