#pragma once

#include <memory>
#include <optional>

#include "software/simulation/firmware_object_deleter.h"

extern "C"
{
#include "firmware/app/primitives/primitive_manager.h"
#include "shared/proto/primitive.nanopb.h"
}

/**
 *  The SimulatorRobot is an abstract class for simulator robot implementations
 */
class SimulatorRobot
{
    friend class SimulatorRobotSingleton;

   public:
    /**
     * Create a new SimulatorRobot
     */
    explicit SimulatorRobot();

    /**
     * Returns the ID of this robot
     *
     * @return the ID of this robot
     */
    virtual unsigned int getRobotId() = 0;

   protected:
    /**
     * Returns the x-position of the robot, in global field coordinates, in meters
     *
     * @return the x-position of the robot, in global field coordinates, in meters
     */
    virtual float getPositionX() = 0;

    /**
     * Returns the y-position of the robot, in global field coordinates, in meters
     *
     * @return the y-position of the robot, in global field coordinates, in meters
     */
    virtual float getPositionY() = 0;

    /**
     * Returns the orientation of the robot, in global field coordinates, in radians
     *
     * @return the orientation of the robot, in global field coordinates, in radians
     */
    virtual float getOrientation() = 0;

    /**
     * Returns the x-velocity of the robot, in global field coordinates, in m/s
     *
     * @return the x-velocity of the robot, in global field coordinates, in m/s
     */
    virtual float getVelocityX() = 0;

    /**
     * Returns the y-velocity of the robot, in global field coordinates, in m/s
     *
     * @return the y-velocity of the robot, in global field coordinates, in m/s
     */
    virtual float getVelocityY() = 0;

    /**
     * Returns the angular velocity of the robot, in rad/s
     *
     * @return the angular of the robot, in rad/s
     */
    virtual float getVelocityAngular() = 0;

    /**
     * Returns the battery voltage, in volts
     *
     * @return the battery voltage, in volts
     */
    virtual float getBatteryVoltage() = 0;

    /**
     * Fires the kicker, kicking the ball in the direction the robot is facing
     * at the given speed if the ball is very close to the kicker
     *
     * @param speed_m_per_s How fast to kick the ball, in meters per second
     */
    virtual void kick(float speed_m_per_s) = 0;

    /**
     * Fires the chipper, chipping the ball in the direction the robot is facing
     * for the given distance if the ball is very close to the chipper
     *
     * @param speed_m_per_s How far to chip the ball (the distance to the first bounce)
     * in meters
     */
    virtual void chip(float distance_m) = 0;

    /**
     * Enables autokick on the robot. If the ball touches the kicker, the robot will
     * kick the ball with the given speed.
     *
     * @param speed_m_per_s How fast to kick the ball in meters per second when
     * the kicker is fired
     */
    virtual void enableAutokick(float speed_m_per_s) = 0;

    /**
     * Enables autochip on the robot. If the ball touches the chipper, the robot will
     * chip the ball the given distance.
     *
     * @param speed_m_per_s How far to chip the ball (distance to the first bounce)
     * when the chipper is fired
     */
    virtual void enableAutochip(float distance_m) = 0;

    /**
     * Disables autokick
     */
    virtual void disableAutokick() = 0;

    /**
     * Disables autochip
     */
    virtual void disableAutochip() = 0;

    /**
     * Returns true if autokick is enabled and false otherwise
     *
     * @return true if autokick is enabled and false otherwise
     */
    virtual bool isAutokickEnabled() = 0;

    /**
     * Returns true if autochip is enabled and false otherwise
     *
     * @return true if autochip is enabled and false otherwise
     */
    virtual bool isAutochipEnabled() = 0;

    /**
     * Sets the speed of the dribbler
     *
     * @param rpm The rpm to set for the dribbler
     */
    virtual void setDribblerSpeed(uint32_t rpm) = 0;

    /**
     * Makes the dribbler coast until another operation is applied to it
     */
    virtual void dribblerCoast() = 0;

    /**
     * Returns the temperature of the dribbler, in degrees C
     *
     * @return the temperature of the dribbler, in degrees C
     */
    virtual unsigned int getDribblerTemperatureDegC() = 0;

    /**
     * Gets the motor speed for the wheel, in RPM
     */
    virtual float getMotorSpeedFrontLeft()  = 0;
    virtual float getMotorSpeedBackLeft()   = 0;
    virtual float getMotorSpeedBackRight()  = 0;
    virtual float getMotorSpeedFrontRight() = 0;

    /**
     * Sets the current primitive this robot is running to a new one
     *
     * @param firmware_world The world to run the primitive in
     * @param primitive_msg The primitive to start
     */
    virtual void startNewPrimitive(std::shared_ptr<FirmwareWorld_t> firmware_world,
                                   const TbotsProto_Primitive& primitive_msg) = 0;

    /**
     * Runs the current primitive
     *
     * @param world The world to run the primitive in
     */
    virtual void runCurrentPrimitive(std::shared_ptr<FirmwareWorld_t> firmware_world) = 0;

    std::optional<float> autokick_speed_m_per_s;
    std::optional<float> autochip_distance_m;
    std::unique_ptr<PrimitiveManager, FirmwarePrimitiveManagerDeleter> primitive_manager;
};
