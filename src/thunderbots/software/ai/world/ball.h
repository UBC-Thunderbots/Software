#pragma once

#include <thunderbots_msgs/Ball.h>
#include "geom/point.h"

class Ball final
{
   public:
    // The approximate radius of the ball according to the SSL rulebook
    static const constexpr double MAX_RADIUS = 0.0215;

    /**
     * Creates a new ball
     */
    explicit Ball();

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
     * Updates the ball with new data from a Ball message. This updates the current data
     * as well as the predictive models
     *
     * @param ball_msg The message containing the new data to update the Ball with
     */
    void update(const thunderbots_msgs::Ball& ball_msg);

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
    Point position_;
    Vector velocity_;
};
