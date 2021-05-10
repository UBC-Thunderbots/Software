#pragma once

#include "software/simulation/force_wheel_simulator_robot.h"
#include "software/simulation/simulator_robot_singleton.h"

/**
 * This class acts as a wrapper around a ForceWheelSimulatorRobot and extends the common
 * functionality provided by the SimulatorRobotSingleton class
 */
class ForceWheelSimulatorRobotSingleton : public SimulatorRobotSingleton
{
   public:
    /**
     * Sets the ForceWheelSimulatorRobot being controlled by this class
     *
     * @param robot The ForceWheelSimulatorRobot being controlled by this class
     * @param field_side The side of the field being defended by the robots using
     * this class
     */
    static void setSimulatorRobot(std::shared_ptr<ForceWheelSimulatorRobot> robot,
                                  FieldSide field_side);

    /**
     * Creates a FirmwareRobot_t with functions bound to the static functions in this
     * class. Only one FirmwareRobot_t needs to be created to control all robots, since
     * calling setSimulatorRobot will simply change the implementations of the bound
     * functions to act as if the new robot was being controlled.
     *
     * @return a FirmwareRobot_t that is bound to whatever ForceWheelSimulatorRobot this
     * Singleton is controlling
     */
    static std::unique_ptr<FirmwareRobot_t, FirmwareRobotDeleter> createFirmwareRobot();

   private:
    /**
     * Applies the given force to the wheel
     *
     * @param force_in_newtons the force to apply to the wheel
     */
    static void applyWheelForceFrontLeft(float force_in_newtons);
    static void applyWheelForceBackLeft(float force_in_newtons);
    static void applyWheelForceBackRight(float force_in_newtons);
    static void applyWheelForceFrontRight(float force_in_newtons);

    /**
     * Helper function that checks if the pointer to the simulator_robot is valid before
     * calling the given function. If the simulator_robot is invalid, a warning is logged
     * and a default value is returned.
     *
     * @param func The function to perform on the simulator robot
     */
    template <class T>
    static T checkValidAndExecute(
        std::function<T(std::shared_ptr<ForceWheelSimulatorRobot>)> func)
    {
        if (force_wheel_simulator_robot)
        {
            return func(force_wheel_simulator_robot);
        }
        LOG(WARNING)
            << "ForceWheelSimulatorRobotSingleton called without setting the ForceWheelSimulatorRobot first"
            << std::endl;
        return static_cast<T>(0);
    }

    // The simulator robot being controlled by this class
    static std::shared_ptr<ForceWheelSimulatorRobot> force_wheel_simulator_robot;
};
