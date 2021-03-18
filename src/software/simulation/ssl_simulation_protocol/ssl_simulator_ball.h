#pragma once

#include "software/simulation/simulator_ball.h"

/**
 * The SslSimulatorBall class implements the ball abstraction for SSL simulation protocol
 */
class SslSimulatorBall : public SimulatorBall
{
  public:
    /**
     * Create a new SslSimulatorBall given a ball state
     *
     * @param ball_state the initial ball state
     */
    explicit SslSimulatorBall(BallState ball_state);

    /**
     * Returns the current position of the ball, in global field coordinates, in meters
     *
     * @return the current position of the ball, in global field coordinates, in meters
     */
    Point position() const override;

    /**
     * Returns the current velocity of the ball, in global field coordinates, in m/s
     *
     * @return the current velocity of the ball, in global field coordinates, in m/s
     */
    Vector velocity() const override;

    /**
     * Sets the current state of the ball
     * 
     * @param p the current position of the ball
     * @param v the current velocity of the ball
     */
    void setState(Point p, Vecotr v);

   private:
    BallState ball_state 
}