#pragma once

#include "software/simulation/simulator_robot.h"

class ForceWheelSimulatorRobot : public SimulatorRobot
{
    friend class ForceWheelSimulatorRobotSingleton;

   protected:
    /**
     * Applies the given force to the wheel
     *
     * @param force_in_newtons the force to apply to the wheel
     */
    virtual void applyWheelForceFrontLeft(float force_in_newtons)  = 0;
    virtual void applyWheelForceBackLeft(float force_in_newtons)   = 0;
    virtual void applyWheelForceBackRight(float force_in_newtons)  = 0;
    virtual void applyWheelForceFrontRight(float force_in_newtons) = 0;
};
