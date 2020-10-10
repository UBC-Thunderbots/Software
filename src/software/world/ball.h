#pragma once

#include <memory>
#include <optional>

#include "software/time/timestamp.h"
#include "software/world/ball_model/ball_model.h"
#include "software/world/ball_state.h"

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
     * @param timestamp the initial timestamp
     */
    explicit Ball(const BallState &initial_state, const Timestamp &timestamp);

    /**
     * Returns the current state of the ball
     *
     * @return BallState
     */
    BallState currentState() const;

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
     * @param new_timestamp the new timestamp
     */
    void updateState(const BallState &new_state, const Timestamp &new_timestamp);

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
    BallState current_state_;
    Timestamp timestamp_;
    std::shared_ptr<BallModel> ball_model_;
};
