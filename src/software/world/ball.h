#pragma once

#include <optional>

#include "software/geom/point.h"
#include "software/geom/vector.h"
#include "software/time/timestamp.h"
#include "software/world/ball_model/ball_model.h"
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
     */
    explicit Ball(const Point &position, const Vector &velocity,
                  const Timestamp &timestamp);

    /**
     * Creates a new ball with the given initial state
     *
     * @param initial_state The initial state of the ball
     */
    explicit Ball(const TimestampedBallState &initial_state);

    /**
     * Returns the current state of the ball
     *
     * @return TimestampedBallState
     */
    TimestampedBallState currentState() const;

    /**
     * Returns the ball model to predict future states
     *
     * @return ball model
     */
    const std::shared_ptr<BallModel> &ballModel() const;

    /**
     * Updates the ball with new data
     *
     * @param new_state the new state of the ball
     */
    void updateState(const TimestampedBallState &new_state);

    /**
     * Returns the current timestamp for when this ball
     *
     * @return the current timestamp
     */
    Timestamp timestamp() const;

    /**
     * Returns the current position of the ball
     *
     * @return the current position of the ball
     */
    Point position() const;

    /**
     * Returns the current velocity of the ball
     *
     * @return the current velocity of the ball
     */
    Vector velocity() const;

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
    TimestampedBallState current_state_;
    std::shared_ptr<BallModel> ball_model_;
};
