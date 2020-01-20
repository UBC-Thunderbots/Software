#pragma once

#include <cinttypes>
#include <optional>

#include "software/backend/simulation/physics/physics_robot.h"
extern "C"
{
#include "app/world/firmware_robot.h"
}

/**
 * This class acts as a wrapper around a PhysicsRobot so that the PhysicsRobot
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
class SimulatorRobotSingleton
{
   public:
    /**
     * Sets the ID of the robot being controlled by this class
     *
     * @param id The ID of the robot to control
     */
    static void setRobotId(unsigned int id);

    /**
     * Sets the PhysicsRobots that can be controlled by this class
     *
     * @param robots the PhysicsRobots that can be controlled by this class
     */
    static void setPhysicsRobots(const std::vector<std::weak_ptr<PhysicsRobot>>& robots);

    /**
     * Creates a FirmwareRobot corresponding to the current PhysicsRobot
     *
     * @return a FirmwareRobot corresponding to the current PhysicsRobot
     */
    static FirmwareRobot_t* createFirmwareRobot();

   private:
    /* Position functions */
    /**
     * Returns the x-position of the robot, in global field coordinates
     *
     * @return the x-position of the robot, in global field coordinates
     */
    static float getPositionX();

    /**
     * Returns the y-position of the robot, in global field coordinates
     *
     * @return the y-position of the robot, in global field coordinates
     */
    static float getPositionY();

    /* Chicker functions */
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

    /* Dribbler functions */
    /**
     * Sets the speed of the dribbler
     *
     * @param rpm The rpm to set for the dribbler
     */
    static void setDribblerSpeed(uint32_t rpm);

    /**
     * Returns the temperature of the dribbler, in degrees C
     *
     * @return the temperature of the dribbler, in degrees C
     */
    static unsigned int getDribblerTemperatureDegC();

    /* Wheel functions */
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
     * Returns the PhysicsRobot currently selected from the list of controllable
     * physics_robots by the current robot_id
     *
     * @return the PhysicsRobot currently selected to be controlled. Returns an empty
     * weak_ptr if the robot_id is not set, or a physics robot with a matching ID
     * cannot be found.
     */
    static std::weak_ptr<PhysicsRobot> getCurrentPhysicsRobot();

    // The id of the robot currently being controlled by this class
    static std::optional<unsigned int> robot_id;
    // All the physics robots this class can control
    static std::vector<std::weak_ptr<PhysicsRobot>> physics_robots;
};
