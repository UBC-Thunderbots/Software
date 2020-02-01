#pragma once

#include <cinttypes>
#include <optional>

#include "software/backend/simulation/physics/physics_robot.h"
extern "C"
{
#include "firmware/main/app/world/firmware_robot.h"
}

// TODO: These are all hardcoded values copied from firmware/main/physics/physics.h
// and firmware/main/control/control.h
// They should be replaced with the proper constants once firmware cleanup is done
#define GEAR_RATIO 0.5143f  // define as speed multiplication from motor to wheel
#define WHEEL_RADIUS 0.0254f
#define WHEEL_SLIP_VOLTAGE_LIMIT 4.25f  // Voltage where wheel slips (acceleration cap)
#define RPM_TO_VOLT (1.0f / 374.0f)     // motor RPM to back EMF
#define PHASE_RESISTANCE 1.6f           // adjust this number as calculated
#define CURRENT_PER_TORQUE 39.21f       // from motor data sheet (1/25.5 mNm)
#define ROBOT_POINT_MASS 2.48f
#define ROBOT_RADIUS 0.085f
#define INERTIAL_FACTOR 0.37f
#define ROT_MASS (INERTIAL_FACTOR * ROBOT_POINT_MASS)
#define INERTIA (ROT_MASS * ROBOT_RADIUS * ROBOT_RADIUS)
#define JERK_LIMIT 40.0f  //(m/s^3)


/**
 * Because the FirmwareRobot_t struct is defined in the .c file (rather than the .h file),
 * C++ considers it an incomplete type and is unable to use it with smart pointers
 * because it doesn't know the size of the object. Therefore we need to create our own
 * "Deleter" class we can provide to the smart pointers to handle that instead.
 *
 * See https://en.cppreference.com/w/cpp/memory/unique_ptr/unique_ptr for more info and
 * examples
 */
struct FirmwareRobotDeleter
{
    void operator()(FirmwareRobot_t* firmware_robot) const
    {
        app_firmware_robot_destroy(firmware_robot);
    };
};


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
    static std::unique_ptr<FirmwareRobot_t, FirmwareRobotDeleter> createFirmwareRobot();

   private:
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

    /* Dribbler functions */
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
