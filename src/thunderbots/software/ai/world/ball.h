#ifndef AI_WORLD_BALL_H_
#define AI_WORLD_BALL_H_

#include "geom/point.h"

class Ball
{
   public:
    // The approximate radius of the ball according to the SSL rulebook
    static constexpr double MAX_RADIUS = 0.0215;

    /**
     * Creates a new ball
     */
    explicit Ball();

    /**
     * Updates the ball with new data.
     *
     * @param new_position the new position of the ball.
     * @param new_velocity the new velocity of the ball.
     */
    void update(Point &new_position, Point &new_velocity);

    /**
     * Get the predicted position of the ball at a time relative to the current time.
     * Using the default of 0 will give the current position of the ball.
     *
     * @param time_delta the amount of time in seconds forward to predict
     *
     * @return the predicted position of the ball.
     */
    Point position(double time_delta = 0.0) const;

    /**
     * Get the predicted velocity of the ball at a time relative to the current time.
     * Using the default of 0 will give the current velocity of the ball.
     *
     * @param time_delta the amount of time in seconds forwards to predict
     *
     * @return the predicted velocity of the ball;
     */
    Point velocity(double time_delta = 0.0) const;

   private:
    Point position_;
    Point velocity_;
};

#endif  // AI_WORLD_BALL_H_
