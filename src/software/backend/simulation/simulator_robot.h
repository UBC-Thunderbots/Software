#pragma once

#include <cinttypes>
#include <memory>
#include "software/backend/simulation/physics/physics_robot.h"
#include "software/backend/simulation/physics/physics_ball.h"
#include "software/ai/primitive/primitive.h"

// TODO: comment
class SimulatorRobot {
public:
    explicit SimulatorRobot(std::weak_ptr<PhysicsRobot> physics_robot);
    explicit SimulatorRobot() = default;

    /**
     * Returns the ID of this robot
     *
     * @return the ID of this robot
     */
    unsigned int getRobotId();

    /**
     * Returns the x-position of the robot, in global field coordinates, in meters
     *
     * @return the x-position of the robot, in global field coordinates, in meters
     */
    float getPositionX();

    /**
     * Returns the y-position of the robot, in global field coordinates, in meters
     *
     * @return the y-position of the robot, in global field coordinates, in meters
     */
    float getPositionY();

    /**
     * Returns the current position of the robot, in global field coordinates, in meters
     *
     * @return the current position of the robot, in global field coordinates, in meters
     */
    Point position();

    /**
     * Returns the orientation of the robot, in global field coordinates, in radians
     *
     * @return the orientation of the robot, in global field coordinates, in radians
     */
    float getOrientation();

    /**
     * Returns the x-velocity of the robot, in global field coordinates, in m/s
     *
     * @return the x-velocity of the robot, in global field coordinates, in m/s
     */
    float getVelocityX();

    /**
     * Returns the y-velocity of the robot, in global field coordinates, in m/s
     *
     * @return the y-velocity of the robot, in global field coordinates, in m/s
     */
    float getVelocityY();

    /**
     * Returns the current linear velocity of the robot, in global field coordinates, in m / s
     *
     * @return the current linear velocity of the robot, in global field coordinates, in m / s
     */
    Vector velocity();

    /**
     * Returns the angular velocity of the robot, in rad/s
     *
     * @return the angular of the robot, in rad/s
     */
    float getVelocityAngular();

    /**
     * Returns the battery voltage, in volts
     *
     * @return the battery voltage, in volts
     */
    float getBatteryVoltage();

    /**
     * Fires the kicker, kicking the ball in the direction the robot is facing
     * at the given speed if the ball is very close to the kicker
     *
     * @param speed_m_per_s How fast to kick the ball, in meters per second
     */
    void kick(float speed_m_per_s);

    /**
     * Fires the chipper, chipping the ball in the direction the robot is facing
     * for the given distance if the ball is very close to the chipper
     *
     * @param speed_m_per_s How far to chip the ball (the distance to the first bounce)
     * in meters
     */
    void chip(float distance_m);

    /**
     * Enables autokick on the robot. If the ball touches the kicker, the robot will
     * kick the ball with the given speed.
     *
     * @param speed_m_per_s How fast to kick the ball in meters per second when
     * the kicker is fired
     */
    void enableAutokick(float speed_m_per_s);

    /**
     * Enables autochip on the robot. If the ball touches the chipper, the robot will
     * chip the ball the given distance.
     *
     * @param speed_m_per_s How far to chip the ball (distance to the first bounce)
     * when the chipper is fired
     */
    void enableAutochip(float distance_m);

    /**
     * Disables autokick
     */
    void disableAutokick();

    /**
     * Disables autochip
     */
    void disableAutochip();

    /**
     * Sets the speed of the dribbler
     *
     * @param rpm The rpm to set for the dribbler
     */
    void setDribblerSpeed(uint32_t rpm);

    /**
     * Returns the dribbler speed in rpm
     *
     * @return the dribbler speed in rpm
     */
    uint32_t getDribblerSpeed() const;

    /**
     * Makes the dribbler coast until another operation is applied to it
     */
    void dribblerCoast();

    /**
     * Returns the temperature of the dribbler, in degrees C
     *
     * @return the temperature of the dribbler, in degrees C
     */
    unsigned int getDribblerTemperatureDegC();

    /**
     * Applies the given force to the wheel
     *
     * @param force_in_newtons the force to apply to the wheel
     */
    void applyWheelForceFrontLeft(float force_in_newtons);
    void applyWheelForceBackLeft(float force_in_newtons);
    void applyWheelForceBackRight(float force_in_newtons);
    void applyWheelForceFrontRight(float force_in_newtons);

    /**
     * Gets the motor speed for the wheel, in RPM
     */
    float getMotorSpeedFrontLeft();
    float getMotorSpeedBackLeft();
    float getMotorSpeedBackRight();
    float getMotorSpeedFrontRight();

private:
    // TODO: comment
    void onChickerBallContact(PhysicsRobot* robot, PhysicsBall* ball);
    void onDribblerBallContact(PhysicsRobot* robot, PhysicsBall* ball);
    void onDribblerBallStartContact(PhysicsRobot* robot, PhysicsBall* ball);
    void onDribblerBallEndContact(PhysicsRobot* robot, PhysicsBall* ball);

    std::weak_ptr<PhysicsRobot> physics_robot;
    std::optional<double> autokick_speed_m_per_s;
    std::optional<double> autochip_distance_m;
    uint32_t dribbler_rpm;

    PhysicsBall* ball_in_dribbler_area;

};