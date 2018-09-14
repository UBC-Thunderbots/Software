#pragma once

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
     */
    explicit Ball(Point position = Point(), Vector velocity = Vector());

    /**
     * Updates the ball with new data, updating the current data as well as the predictive
     * model
     *
     * @param new_position the new position of the ball, defined in metres
     * @param new_velocity the new velocity of the ball, defined in metres per second
     */
    void update(const Point& new_position, const Vector& new_velocity);

    /**
     * Updates the ball with new data, updating the current data as well as the predictive
     * model
     *
     * @param new_ball_data A ball containing new ball data
     */
    void update(const Ball& new_ball_data);

    /**
     * Get the predicted position of the ball at a time relative to the current time.
     * Using the default of 0 will give the current position of the ball.
     *
     * @param time_delta the amount of time in seconds forward to predict
     *
     * @return the predicted position of the ball, defined in metres
     */
    Point position(double time_delta = 0.0) const;

    /**
     * Get the predicted velocity of the ball at a time relative to the current time.
     * Using the default of 0 will give the current velocity of the ball.
     *
     * @param time_delta the amount of time in seconds forwards to predict
     *
     * @return the predicted velocity of the ball, defined in metres per second
     */
    Vector velocity(double time_delta = 0.0) const;

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
};
