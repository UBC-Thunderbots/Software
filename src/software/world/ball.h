#pragma once

#include <memory>
#include <optional>

#include "proto/world.pb.h"
#include "software/time/timestamp.h"
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
     * @param acceleration acceleration being applied to the ball
     */
    explicit Ball(const Point &position, const Vector &velocity,
                  const Timestamp &timestamp, const Vector &acceleration = Vector(0, 0));

    /**
     * Creates a new ball with the given initial state
     *
     * @param initial_state The initial state of the ball
     * @param timestamp the initial timestamp
     * @param acceleration acceleration being applied to the ball
     */
    explicit Ball(const BallState &initial_state, const Timestamp &timestamp,
                  const Vector &acceleration = Vector(0, 0));

    /**
     * Creates a new ball based on the TbotsProto::Ball protobuf representation
     *
     * @param ball_proto The TbotsProto::Ball protobuf which this ball should be based on
     */
    explicit Ball(const TbotsProto::Ball &ball_proto);

    /**
     * Returns the current state of the ball
     *
     * @return BallState
     */
    BallState currentState() const;

    /**
     * Updates the ball with new data
     *
     * @param new_state the new state of the ball
     * @param new_timestamp the new timestamp
     * @param new_acceleration acceleration being applied to the ball
     */
    void updateState(const BallState &new_state, const Timestamp &new_timestamp,
                     const Vector &new_acceleration = Vector(0, 0));

    /**
     * Returns the current timestamp for this ball
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
     * Returns the current acceleration of the ball
     *
     * @return the current acceleration of the ball
     */
    Vector acceleration() const;

    /**
     * Returns the estimated state of the ball at the specified amount of time in the
     * future
     *
     * @param duration_in_future The Duration into the future at which to predict the
     * ball's position
     *
     * @return The future state of the ball
     */
    BallState estimateFutureState(const Duration &duration_in_future) const;


    /**
     * Returns the estimated time it will take the ball to move a distance based on its
     * current state
     *
     * @param distance the distance from the current position of the ball
     *
     * @return Duration of time to move distance
     */
    std::optional<Duration> getTimeToMoveDistance(double distance) const;

    /**
     * Software approximation that finds if a ball has been kicked, regardless of whether
     * the kick was a pass, shot, or chip.
     *
     * @param expected_kick_direction The direction that we expect the ball to be kicked
     * towards
     * @param min_kick_speed The minimum speed of the ball to be considered a kick, in
     * metres per second
     *
     * @return True if ball was kicked in the approximate direction we expect, false
     * otherwise
     */
    bool hasBallBeenKicked(const Angle &expected_kick_direction,
                           double min_kick_speed = 0.5) const;

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
    Vector acceleration_;  // used to predict future states
};
