#pragma once

#include <functional>
#include <memory>

#include "software/simulation/physics/physics_ball.h"
#include "software/simulation/simulator_ball.h"

/**
 * The PhysicsSimulatorBall class acts as a wrapper for a PhysicsBall that deals with more
 * logic-focused elements for simulation.
 */
class PhysicsSimulatorBall : public SimulatorBall
{
   public:
    /**
     * Create a new PhysicsSimulatorBall given a PhysicsBall
     *
     * @param physics_ball the PhysicsBall to simulate and control
     */
    explicit PhysicsSimulatorBall(std::weak_ptr<PhysicsBall> physics_ball);
    explicit PhysicsSimulatorBall() = delete;

    Point position() const override;

    Vector velocity() const override;

   private:
    /**
     * Helper functions that check if the current pointer to the physics_ball is valid
     * before calling the given function. If the physics_ball is invalid, a warning is
     * logged and a default value is returned;
     *
     * @tparam RET_VAL the return value of the function to execute
     * @param func The function to perform on the physics ball
     */
    template <class RET_VAL>
    RET_VAL checkValidAndExecute(
        std::function<RET_VAL(const std::shared_ptr<PhysicsBall>)> func) const
    {
        if (auto ball = physics_ball.lock())
        {
            return func(ball);
        }
        LOG(WARNING) << "PhysicsSimulatorBall being used with invalid PhysicsBall"
                     << std::endl;
        return RET_VAL();
    }

    std::weak_ptr<PhysicsBall> physics_ball;
};
