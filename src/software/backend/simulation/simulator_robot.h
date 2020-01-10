#include <cinttypes>
extern "C" {
#include "app/world/firmware_robot.h"
}

/**
 * This class acts as a wrapper around a PhysicsRobot so that the PhysicsRobot
 * can provide the interface of a FirmwareRobot
 */
class SimulatorRobot {
public:
    /**
     * Sets the ID of the robot being controlled by this class
     *
     * @param id The ID of the robot to control
     */
    static void setRobotId(unsigned int id);

    /**
     * Creates a FirmwareRobot corresponding to the current PhysicsRobot
     *
     * @return a FirmwareRobot corresponding to the current PhysicsRobot
     */
    static FirmwareRobot_t* createFirmwareRobot();

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

private:
    // The id of the robot currently being controlled by this class
    static unsigned int robot_id;
};
unsigned int SimulatorRobot::robot_id = 0;