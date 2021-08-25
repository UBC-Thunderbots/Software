#pragma once

#include <cinttypes>
#include <functional>
#include <optional>

#include "software/logger/logger.h"
#include "software/simulation/er_force_simulator_robot.h"
#include "software/world/field.h"

extern "C"
{
#include "firmware/app/world/chicker.h"
#include "firmware/app/world/dribbler.h"
#include "firmware/app/world/firmware_robot.h"
#include "firmware/app/world/force_wheel.h"
#include "firmware/shared/physics.h"
#include "shared/proto/primitive.nanopb.h"
#include "shared/proto/robot_log_msg.nanopb.h"
#include "software/simulation/firmware_object_deleter.h"
}

/**
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
class ErForceSimulatorRobotSingleton
{
   public:
    /**
     * Starts a new primitive on the SimulatorRobot currently being controlled by this
     * class
     *
     * @param firmware_world The world to run the primitive in
     * @param primitive_msg The primitive to start
     */
    static void startNewPrimitiveOnCurrentSimulatorRobot(
        std::shared_ptr<FirmwareWorld_t> firmware_world,
        const TbotsProto_Primitive& primitive_msg);

    /**
     * Runs the current primitive on the SimulatorRobot currently being controlled by this
     * class
     *
     * @param firmware_world The world to run the primitive in
     */
    static void runPrimitiveOnCurrentSimulatorRobot(
        std::shared_ptr<FirmwareWorld_t> firmware_world);

    /**
     * Handler for the firmware logger, runs on every TLOG_* event. Since we can have
     * two robots with the same ID, we have a log handler for each robot color
     *
     * @param the nanopb RobotLog proto logged by a robot
     */
    static void handleBlueRobotLogProto(TbotsProto_RobotLog log);
    static void handleYellowRobotLogProto(TbotsProto_RobotLog log);

    /**
     * Returns the x-position of the robot, in global field coordinates, in meters
     *
     * @return the x-position of the robot, in global field coordinates, in meters
     */
    static float getPositionX();

    /**
     * Returns the y-position of the robot, in global field coordinates, in meters
     *
     * @return the y-position of the robot, in global field coordinates, in meters
     */
    static float getPositionY();

    /**
     * Returns the orientation of the robot, in global field coordinates, in radians
     *
     * @return the orientation of the robot, in global field coordinates, in radians
     */
    static float getOrientation();

    /**
     * Returns the x-velocity of the robot, in global field coordinates, in m/s
     *
     * @return the x-velocity of the robot, in global field coordinates, in m/s
     */
    static float getVelocityX();

    /**
     * Returns the y-velocity of the robot, in global field coordinates, in m/s
     *
     * @return the y-velocity of the robot, in global field coordinates, in m/s
     */
    static float getVelocityY();

    /**
     * Returns the angular velocity of the robot, in rad/s
     *
     * @return the angular of the robot, in rad/s
     */
    static float getVelocityAngular();

    /**
     * Returns the battery voltage, in volts
     *
     * @return the battery voltage, in volts
     */
    static float getBatteryVoltage();

    /**
     * Fires the kicker, kicking the ball in the direction the robot is facing
     * at the given speed if the ball is very close to the kicker
     *
     * @param speed_m_per_s How fast to kick the ball, in meters per second
     */
    static void kick(float speed_m_per_s);

    /**
     * Fires the chipper, chipping the ball in the direction the robot is facing
     * for the given distance if the ball is very close to the chipper
     *
     * @param speed_m_per_s How far to chip the ball (the distance to the first bounce)
     * in meters
     */
    static void chip(float distance_m);

    /**
     * Enables autokick on the robot. If the ball touches the kicker, the robot will
     * kick the ball with the given speed.
     *
     * @param speed_m_per_s How fast to kick the ball in meters per second when
     * the kicker is fired
     */
    static void enableAutokick(float speed_m_per_s);

    /**
     * Enables autochip on the robot. If the ball touches the chipper, the robot will
     * chip the ball the given distance.
     *
     * @param speed_m_per_s How far to chip the ball (distance to the first bounce)
     * when the chipper is fired
     */
    static void enableAutochip(float distance_m);

    /**
     * Disables autokick
     */
    static void disableAutokick();

    /**
     * Disables autochip
     */
    static void disableAutochip();

    /**
     * Sets the speed of the dribbler
     *
     * @param rpm The rpm to set for the dribbler
     */
    static void setDribblerSpeed(uint32_t rpm);

    /**
     * Makes the dribbler coast until another operation is applied to it
     */
    static void dribblerCoast();

    /**
     * Returns the temperature of the dribbler, in degrees C
     *
     * @return the temperature of the dribbler, in degrees C
     */
    static unsigned int getDribblerTemperatureDegC();

    /**
     * Gets the motor speed for the wheel, in RPM
     */
    static float getMotorSpeedFrontLeft();
    static float getMotorSpeedBackLeft();
    static float getMotorSpeedBackRight();
    static float getMotorSpeedFrontRight();

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
     * Helper function for handling robot logs with given team color string
     *
     * @param log The nanopb RobotLog proto logged by a robot
     * @param robot_colour The color of the robot logging
     */
    static void handleRobotLogProto(TbotsProto_RobotLog log,
                                    const std::string& robot_colour);

    /**
     * Sets the ErForceSimulatorRobot being controlled by this class
     *
     * @param robot The ErForceSimulatorRobot being controlled by this class
     */
    static void setSimulatorRobot(std::shared_ptr<ErForceSimulatorRobot> robot);

    /**
     * Creates a FirmwareRobot_t with functions bound to the static functions in this
     * class. Only one FirmwareRobot_t needs to be created to control all robots, since
     * calling setSimulatorRobot will simply change the implementations of the bound
     * functions to act as if the new robot was being controlled.
     *
     * @return a FirmwareRobot_t that is bound to whatever ErForceSimulatorRobot
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
        std::function<T(std::shared_ptr<ErForceSimulatorRobot>)> func)
    {
        if (er_force_simulator_robot)
        {
            return func(er_force_simulator_robot);
        }
        LOG(WARNING)
            << "ErForceSimulatorRobotSingleton called without setting the ErForceSimulatorRobot first"
            << std::endl;
        return static_cast<T>(0);
    }

    // The simulator robot being controlled by this class
    static std::shared_ptr<ErForceSimulatorRobot> er_force_simulator_robot;
};
