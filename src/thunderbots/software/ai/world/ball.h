#pragma once

#include <thunderbots_msgs/Ball.h>
#include "geom/point.h"

class Ball final
{
   public:
    // The approximate radius of the ball according to the SSL rulebook
    static constexpr double MAX_RADIUS = 0.0215;

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
    void update(Point &new_position, Vector2 &new_velocity);

    /**
     * Updates the ball with new data from a Ball message. This updates the current data
     * as well as the predictive models
     *
     * @param ball_msg The message containing the new data to update the Ball with
     */
    void update(const thunderbots_msgs::Ball &ball_msg);

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
    Vector2 velocity(double time_delta = 0.0) const;

   private:
    Point position_;
    Vector2 velocity_;
};
