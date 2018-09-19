#pragma once

#include <chrono>
#include "geom/point.h"

class Ball final
{
   public:
    /**
     * Creates a new ball with the given position and velocity
     *
     * @param position The position of the ball, with coordinates in metres.
     * Default is (0, 0)
     * @param velocity The velocity of the ball, in metres per second. Default is (0, 0)
     * @paragraph last_update_timestamp A timestamp of when this ball's information was
     * last updated. Default is the current time.
     */
    explicit Ball(Point position = Point(), Vector velocity = Vector(),
                  std::chrono::time_point<std::chrono::steady_clock>
                      last_update_timestamp = std::chrono::steady_clock::now());

    /**
     * Updates the ball with new data, updating the current data as well as the predictive
     * model
     *
     * @param new_position the new position of the ball, defined in metres
     * @param new_velocity the new velocity of the ball, defined in metres per second
     * @param timestamp The timestamp for the time at which this update is taking place.
     * The timestamp must be >= the ball's latest update timestamp
     */
    void update(const Point& new_position, const Vector& new_velocity,
                std::chrono::time_point<std::chrono::steady_clock> timestamp);

    /**
     * Updates the ball with new data, updating the current data as well as the predictive
     * model
     *
     * @param new_ball_data A ball containing new ball data
     */
    void update(const Ball& new_ball_data);

    /**
     * Updates the ball's state to match what it would be at the given timestamp. The
     * timestamp must be >= the ball's last update timestamp.
     *
     * @param timestamp The timestamp at which to update the ball's state to
     */
    void updateState(std::chrono::time_point<std::chrono::steady_clock> timestamp);

    /**
     * Returns the timestamp for when this ball's data was last updated
     *
     * @return the timestamp for when this ball's data was last updated
     */
    std::chrono::time_point<std::chrono::steady_clock> lastUpdateTimestamp() const;

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
     * @param milliseconds_in_future The relative amount of time in the future
     * (in milliseconds) at which to predict the ball's position. Value must be >= 0.
     * For example, a value of 1500 would return the estimated position of the ball
     * 1.5 seconds in the future.
     *
     * @return the estimated position of the ball at the given number of milliseconds
     * in the future. Coordinates are in metres.
     */
    Point estimatePositionAtFutureTime(
        const std::chrono::milliseconds& milliseconds_in_future) const;

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
     * @param milliseconds_in_future The relative amount of time in the future
     * (in milliseconds) at which to predict the ball's velocity. Value must be >= 0.
     * For example, a value of 1500 would return the estimated velocity of the ball
     * 1.5 seconds in the future.
     *
     * @return the estimated velocity of the ball at the given number of milliseconds
     * in the future. Coordinates are in metres.
     */
    Vector estimateVelocityAtFutureTime(
        const std::chrono::milliseconds& milliseconds_in_future) const;

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
    // The current position of the ball, with coordinates in metres
    Point position_;
    // The current velocity of the ball, in metres per second
    Vector velocity_;
    // The timestamp for when this ball was last updated
    std::chrono::time_point<std::chrono::steady_clock> last_update_timestamp;
};
