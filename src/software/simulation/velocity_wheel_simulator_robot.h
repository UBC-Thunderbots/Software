#pragma once

#include "software/simulation/simulator_robot.h"

class VelocityWheelSimulatorRobot : public SimulatorRobot
{
    friend class VelocityWheelSimulatorRobotSingleton;

   protected:
    /**
     * Sets the target RPM
     *
     * @param rpm The rpm for the wheel to reach
     */
    virtual void setTargetRPMFrontLeft(float rpm)  = 0;
    virtual void setTargetRPMBackLeft(float rpm)   = 0;
    virtual void setTargetRPMBackRight(float rpm)  = 0;
    virtual void setTargetRPMFrontRight(float rpm) = 0;
};
