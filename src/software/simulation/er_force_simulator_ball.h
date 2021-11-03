#pragma once

#include "software/simulation/simulator_ball.h"
#include "software/world/ball_state.h"

/**
 * The ErForceSimulatorBall class implements the ball abstraction for SSL simulation
 * protocol
 */
class ErForceSimulatorBall : public SimulatorBall
{
   public:
    /**
     * Create a new ErForceSimulatorBall given a ball state
     *
     * @param ball_state the initial ball state
     */
    explicit ErForceSimulatorBall(BallState ball_state);

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
     * Sets the current ball state
     *
     * @param ball_state the current ball state
     */
    void setState(BallState ball_state);

   private:
    BallState ball_state_;
};
