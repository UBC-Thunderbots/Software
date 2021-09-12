#pragma once

#include <cinttypes>
#include <memory>

#include "software/proto/ssl_simulation_robot_control.pb.h"
#include "software/proto/ssl_simulation_robot_feedback.pb.h"
#include "software/simulation/firmware_object_deleter.h"
#include "software/simulation/simulator_robot.h"
#include "software/world/robot_state.h"
#include "src/amun/simulator/simulator.h"

extern "C"
{
#include "firmware/app/primitives/primitive_manager.h"
#include "shared/proto/primitive.nanopb.h"
}

/**
 * Represents a robot in SSL simulation, to be controlled by the SimulatorRobotSingleton
 */
class ErForceSimulatorRobot : public SimulatorRobot
{
    friend class ErForceSimulatorRobotSingleton;

   public:
    /**
     * Create a new ErForceSimulatorRobot given robot id and state
     *
     * @param robot_state the robot state with id
     * @param robot_constants The robot constants
     * @param wheel_constants The wheel constants
     */
    explicit ErForceSimulatorRobot(const RobotStateWithId& robot_state_with_id,
                                   RobotConstants_t robot_constants,
                                   WheelConstants_t wheel_constants);

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
    void setRobotState(const RobotState& robot_state);

    /**
     * Returns the current robot command
     *
     * @return the current robot command
     */
    std::unique_ptr<sslsim::RobotCommand> getRobotCommand();

    /**
     * Sets the robot feedback
     *
     * @param robot_feedback the robot feedback
     */
    void setRobotFeedback(const SSLSimulationProto::RobotFeedback& robot_feedback);

    /**
     * Resets the values for robot command
     */
    void reset();

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
     * Sets the target RPM of the wheel
     *
     * @param rpm the target RPM of the wheel
     */
    void setTargetRPMFrontLeft(float rpm);
    void setTargetRPMBackLeft(float rpm);
    void setTargetRPMBackRight(float rpm);
    void setTargetRPMFrontRight(float rpm);

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
    void coastMotorBackLeft() override;
    void coastMotorBackRight() override;
    void coastMotorFrontLeft() override;
    void coastMotorFrontRight() override;

    /**
     * Sets the motor to brake (act against the current direction of rotation)
     */
    void brakeMotorBackLeft() override;
    void brakeMotorBackRight() override;
    void brakeMotorFrontLeft() override;
    void brakeMotorFrontRight() override;

    /**
     * Sets the current primitive this robot is running to a new one
     *
     * @param firmware_world The world to run the primitive in
     * @param primitive_msg The primitive to start
     */
    void startNewPrimitive(std::shared_ptr<FirmwareWorld_t> firmware_world,
                           const TbotsProto_Primitive& primitive_msg) override;

    /**
     * Runs the current primitive
     *
     * @param world The world to run the primitive in
     */
    void runCurrentPrimitive(std::shared_ptr<FirmwareWorld_t> firmware_world) override;

   private:
    /**
     * Returns the motor speeds for all motors on the robot in rpm
     *
     * @return the motor speeds for all motors on the robot in rpm
     */
    std::array<float, 4> getMotorSpeeds() const;

    // Feedback set at the end of each simulator tick after receiving feedback
    bool dribbler_ball_contact;

    // Robot states
    unsigned int id;
    RobotState robot_state;

    std::optional<float> autokick_speed_m_per_s;
    std::optional<float> autochip_distance_m;

    // Values for robot command
    double wheel_speed_front_right = 0.0;  // [rpm]
    double wheel_speed_front_left  = 0.0;  // [rpm]
    double wheel_speed_back_left   = 0.0;  // [rpm]
    double wheel_speed_back_right  = 0.0;  // [rpm]
    std::optional<float> kick_speed;       // [m/s]
    std::optional<float> kick_angle;       // [degree]
    std::optional<double> dribbler_speed;  // [rpm]

    RobotConstants_t robot_constants;
    WheelConstants_t wheel_constants;
};
