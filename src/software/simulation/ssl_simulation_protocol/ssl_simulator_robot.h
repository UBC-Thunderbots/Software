#pragma once

#include <cinttypes>
#include <memory>

#include "software/simulation/firmware_object_deleter.h"

extern "C"
{
#include "firmware/app/primitives/primitive_manager.h"
#include "shared/proto/primitive.nanopb.h"
}

/**
 * Represents a robot in SSL simulation, to be controlled by the SimulatorRobotSingleton
 */
class SslSimulatorRobot : public SimulatorRobot
{
   public:
    /**
     * Create a new SslSimulatorRobot given robot id and state
     *
     * @param id the robot id
     * @param robot_state the robot state
     */
    explicit SslSimulatorRobot(const RobotId id, const RobotState &robot_state);

    /**
     * Returns the ID of this robot
     *
     * @return the ID of this robot
     */
    unsigned int getRobotId() override;

    /**
     * Sets the robot state
     * 
     * @return the robot state
     */
    void setRobotState(RobotState robot_state); 

    /**
     * Returns the current robot command
     * 
     * @return the current robot command
     */
    SslSimulationProto::RobotCommand getRobotCommand();

    /**
     * Resets the robot command to only contain the robot id
     */
    void resetRobotCommand();

   protected:
    /**
     * Returns the x-position of the robot, in global field coordinates, in meters
     *
     * @return the x-position of the robot, in global field coordinates, in meters
     */
    float getPositionX() override;

    /**
     * Returns the y-position of the robot, in global field coordinates, in meters
     *
     * @return the y-position of the robot, in global field coordinates, in meters
     */
    float getPositionY() override;

    /**
     * Returns the orientation of the robot, in global field coordinates, in radians
     *
     * @return the orientation of the robot, in global field coordinates, in radians
     */
    float getOrientation() override;

    /**
     * Returns the x-velocity of the robot, in global field coordinates, in m/s
     *
     * @return the x-velocity of the robot, in global field coordinates, in m/s
     */
    float getVelocityX() override;

    /**
     * Returns the y-velocity of the robot, in global field coordinates, in m/s
     *
     * @return the y-velocity of the robot, in global field coordinates, in m/s
     */
    float getVelocityY() override;

    /**
     * Returns the angular velocity of the robot, in rad/s
     *
     * @return the angular of the robot, in rad/s
     */
    float getVelocityAngular() override;

    /**
     * Returns the battery voltage, in volts
     *
     * @return the battery voltage, in volts
     */
    float getBatteryVoltage() override;

    /**
     * Fires the kicker, kicking the ball in the direction the robot is facing
     * at the given speed if the ball is very close to the kicker
     *
     * @param speed_m_per_s How fast to kick the ball, in meters per second
     */
    void kick(float speed_m_per_s) override;

    /**
     * Fires the chipper, chipping the ball in the direction the robot is facing
     * for the given distance if the ball is very close to the chipper
     *
     * @param speed_m_per_s How far to chip the ball (the distance to the first bounce)
     * in meters
     */
    void chip(float distance_m) override;

    /**
     * Enables autokick on the robot. If the ball touches the kicker, the robot will
     * kick the ball with the given speed.
     *
     * @param speed_m_per_s How fast to kick the ball in meters per second when
     * the kicker is fired
     */
    void enableAutokick(float speed_m_per_s) override;

    /**
     * Enables autochip on the robot. If the ball touches the chipper, the robot will
     * chip the ball the given distance.
     *
     * @param speed_m_per_s How far to chip the ball (distance to the first bounce)
     * when the chipper is fired
     */
    void enableAutochip(float distance_m) override;

    /**
     * Disables autokick
     */
    void disableAutokick() override;

    /**
     * Disables autochip
     */
    void disableAutochip() override;

    /**
     * Returns true if autokick is enabled and false otherwise
     *
     * @return true if autokick is enabled and false otherwise
     */
    bool isAutokickEnabled() override; 

    /**
     * Returns true if autochip is enabled and false otherwise
     *
     * @return true if autochip is enabled and false otherwise
     */
    bool isAutochipEnabled() override;

    /**
     * Sets the speed of the dribbler
     *
     * @param rpm The rpm to set for the dribbler
     */
    void setDribblerSpeed(uint32_t rpm) override;

    /**
     * Makes the dribbler coast until another operation is applied to it
     */
    void dribblerCoast() override;

    /**
     * Returns the temperature of the dribbler, in degrees C
     *
     * @return the temperature of the dribbler, in degrees C
     */
    unsigned int getDribblerTemperatureDegC() override;

    /**
     * Applies the given force to the wheel
     *
     * @param force_in_newtons the force to apply to the wheel
     */
    void applyWheelForceFrontLeft(float force_in_newtons) override;
    void applyWheelForceBackLeft(float force_in_newtons) override;
    void applyWheelForceBackRight(float force_in_newtons) override;
    void applyWheelForceFrontRight(float force_in_newtons) override;

    /**
     * Gets the motor speed for the wheel, in RPM
     */
    float getMotorSpeedFrontLeft() override;
    float getMotorSpeedBackLeft() override;
    float getMotorSpeedBackRight() override;
    float getMotorSpeedFrontRight() override;

    /**
     * Sets the motor to coast (spin freely)
     */
    virtual void coastMotorBackLeft() override;
    virtual void coastMotorBackRight() override;
    virtual void coastMotorFrontLeft() override;
    virtual void coastMotorFrontRight() override;

    /**
     * Sets the motor to brake (act against the current direction of rotation)
     */
    virtual void brakeMotorBackLeft() override;
    virtual void brakeMotorBackRight() override;
    virtual void brakeMotorFrontLeft() override;
    virtual void brakeMotorFrontRight() override;

    /**
     * Sets the current primitive this robot is running to a new one
     *
     * @param firmware_world The world to run the primitive in
     * @param primitive_msg The primitive to start
     */
    virtual void startNewPrimitive(std::shared_ptr<FirmwareWorld_t> firmware_world,
                           const TbotsProto_Primitive& primitive_msg) override;

    /**
     * Runs the current primitive
     *
     * @param world The world to run the primitive in
     */
    virtual void runCurrentPrimitive(std::shared_ptr<FirmwareWorld_t> firmware_world) override;

  private:
    // The firmware will store the robot command here, to be sent out after each tick of simulator
    // This will be reset at the beginning of every tick to be an empty command except the id
    SslSimulationProto::RobotCommand robot_command;

    std::unique_ptr<PrimitiveManager, FirmwarePrimitiveManagerDeleter> primitive_manager;

    // Robot states
    unsigned int id;
    RobotState robot_state; 
    std::optional<float> autokick_speed_m_per_s;
    std::optional<float> autochip_distance_m;
};