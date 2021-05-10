#pragma once

#include "software/simulation/simulator_robot_singleton.h"
#include "software/simulation/velocity_wheel_simulator_robot.h"

/**
 * This class acts as a wrapper around a VelocityWheelSimulatorRobot and extends the
 * common functionality provided by the SimulatorRobotSingleton class
 */
class VelocityWheelSimulatorRobotSingleton : public SimulatorRobotSingleton
{
   public:
    /**
     * Sets the VelocityWheelSimulatorRobot being controlled by this class
     *
     * @param robot The VelocityWheelSimulatorRobot being controlled by this class
     * @param field_side The side of the field being defended by the robots using
     * this class
     */
    static void setSimulatorRobot(std::shared_ptr<VelocityWheelSimulatorRobot> robot,
                                  FieldSide field_side);

    /**
     * Creates a FirmwareRobot_t with functions bound to the static functions in this
     * class. Only one FirmwareRobot_t needs to be created to control all robots, since
     * calling setSimulatorRobot will simply change the implementations of the bound
     * functions to act as if the new robot was being controlled.
     *
     * @return a FirmwareRobot_t that is bound to whatever VelocityWheelSimulatorRobot
     * this Singleton is controlling
     */
    static std::unique_ptr<FirmwareRobot_t, FirmwareRobotDeleter> createFirmwareRobot();

   private:
    /**
     * Sets the target RPM
     *
     * @param rpm The rpm for the wheel to reach
     */
    static void setTargetRPMFrontLeft(float rpm);
    static void setTargetRPMBackLeft(float rpm);
    static void setTargetRPMBackRight(float rpm);
    static void setTargetRPMFrontRight(float rpm);

    /**
     * Helper function that checks if the pointer to the simulator_robot is valid before
     * calling the given function. If the simulator_robot is invalid, a warning is logged
     * and a default value is returned.
     *
     * @param func The function to perform on the simulator robot
     */
    template <class T>
    static T checkValidAndExecute(
        std::function<T(std::shared_ptr<VelocityWheelSimulatorRobot>)> func)
    {
        if (velocity_wheel_simulator_robot)
        {
            return func(velocity_wheel_simulator_robot);
        }
        LOG(WARNING)
            << "VelocityWheelSimulatorRobotSingleton called without setting the VelocityWheelSimulatorRobot first"
            << std::endl;
        return static_cast<T>(0);
    }

    // The simulator robot being controlled by this class
    static std::shared_ptr<VelocityWheelSimulatorRobot> velocity_wheel_simulator_robot;
};
