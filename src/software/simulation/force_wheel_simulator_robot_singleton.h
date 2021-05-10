#pragma once

#include "software/simulation/force_wheel_simulator_robot.h"
#include "software/simulation/simulator_robot_singleton.h"

/**
 * TODO: update these comments
 * This class acts as a wrapper around a SimulatorRobot so that the SimulatorRobot
 * can provide the interface of a FirmwareRobot.
 *
 * Because our firmware structs rely on C-style function pointers, we
 * cannot use C++ constructs like std::function to provide to the structs.
 * Therefore we use this class's static functions to provide the function
 * pointers, since static functions work as C-style function pointers
 * (because there is no instance associated with them).
 *
 * Whenever a caller needs to get a firmware struct or perform an operation, they
 * need to set which robot they want to control so that all the static functions
 * that have been provided to the firmware struct operate on the correct
 * instantiated object. This is our workaround to maintain and simulate multiple
 * "instances" of robot firmware at once.
 */
class ForceWheelSimulatorRobotSingleton : public SimulatorRobotSingleton
{
   public:
    /**
     * TODO: update these comments
     * Sets the SimulatorRobot being controlled by this class
     *
     * The attributes of the robot, such as position, will be given for the POV of
     * a robot defending the specified field side. Eg. If the robot is at (-1, 2)
     * in real-world coordinates, it's position will be reported as (-1, 2) if
     * the negative field side is specified. On the other hand if the positive
     * field side is specified, the robot's position will be reported as (1, -2).
     *
     * This different behaviour for either field side exists because our firmware
     * expects its knowledge of the world to math our coordinate convention, which is
     * relative to the side of the field the robot is defending. See
     * https://github.com/UBC-Thunderbots/Software/blob/master/docs/software-architecture-and-design.md#coordinates
     * for more information about our coordinate conventions. Because we can't actually
     * change the positions and dynamics of the underlying physics objects, we use this
     * class to enforce this convention for the firmware.
     *
     * @param robot The SimulatorRobot being controlled by this class
     * @param field_side The side of the field being defended by the robots using
     * this class
     */
    static void setSimulatorRobot(std::shared_ptr<ForceWheelSimulatorRobot> robot,
                                  FieldSide field_side);

    /**
     * TODO: update these comments
     * Creates a FirmwareRobot_t with functions bound to the static functions in this
     * class. Only one FirmwareRobot_t needs to be created to control all robots, since
     * calling setSimulatorRobot will simply change the implementations of the bound
     * functions to act as if the new robot was being controlled.
     *
     * @return a FirmwareRobot_t that is bound to whatever SimulatorRobot this Singleton
     * is controlling
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
     * Sets the motor to coast (spin freely)
     */
    static void coastMotorBackLeft();
    static void coastMotorBackRight();
    static void coastMotorFrontLeft();
    static void coastMotorFrontRight();

    /**
     * Sets the motor to brake (act against the current direction of rotation)
     */
    static void brakeMotorBackLeft();
    static void brakeMotorBackRight();
    static void brakeMotorFrontLeft();
    static void brakeMotorFrontRight();

    /**
     * Helper functions that check if the pointer to the simulator_robot is valid before
     * calling the given function. If the simulator_robot is invalid, a warning is logged
     * and a default value is returned.
     *
     * @param func The function to perform on the simulator robot
     */
    // TODO: can this be templated?
    static void checkValidAndExecuteVoid(
        std::function<void(std::shared_ptr<ForceWheelSimulatorRobot>)> func);
    static float checkValidAndReturnFloat(
        std::function<float(std::shared_ptr<ForceWheelSimulatorRobot>)> func);
    static unsigned int checkValidAndReturnUint(
        std::function<unsigned int(std::shared_ptr<ForceWheelSimulatorRobot>)> func);

    // The simulator robot being controlled by this class
    static std::shared_ptr<ForceWheelSimulatorRobot> force_wheel_simulator_robot;
};
