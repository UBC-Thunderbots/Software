#pragma once

#include <cinttypes>
#include <memory>

#include "software/simulation/firmware_object_deleter.h"
#include "software/simulation/physics/physics_ball.h"
#include "software/simulation/physics/physics_robot.h"

extern "C"
{
#include "firmware/app/primitives/primitive_manager.h"
#include "shared/proto/primitive.nanopb.h"
}

/**
 * The PhysicsSimulatorRobot class acts as a wrapper for a PhysicsRobot that deals with more
 * logic-focused elements for simulation, such as whether or not autokick is enabled.
 *
 * All members of this class are intentionally protected or private to force this class
 * to only be controlled by the SimulatorRobotSingleton.
 */
class PhysicsSimulatorRobot : public SimulatorRobot
{
   friend class SimulatorRobotSingleton;

   public:
    /**
     * Create a new SimulatorRobot given a PhysicsRobot
     *
     * @param physics_robot the PhysicsRobot to simulate and control
     */
    explicit SimulatorRobot(std::weak_ptr<PhysicsRobot> physics_robot);
    explicit SimulatorRobot() = delete;

    /**
     * Returns the ID of this robot
     *
     * @return the ID of this robot
     */
    unsigned int getRobotId() override;

    /**
     * Clears balls tracked as being in dribbler area
     */
    void clearBallInDribblerArea() override;

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
     * A function that is called during every physics step for as long as the ball
     * is touching this robot's dribbler
     *
     * @param physics_robot The robot involved in the contact
     * @param physics_ball The ball invovled in the contact
     */
    void onDribblerBallContact(PhysicsRobot* physics_robot, PhysicsBall* physics_ball);

    /**
     * A function that is called during once when the ball starts touching
     * this robot's dribbler.
     *
     * @param physics_robot The robot involved in the contact
     * @param physics_ball The ball invovled in the contact
     */
    void onDribblerBallStartContact(PhysicsRobot* physics_robot,
                                    PhysicsBall* physics_ball);

    /**
     * A function that is called during once when the ball stops touching
     * this robot's dribbler.
     *
     * @param physics_robot The robot involved in the contact
     * @param physics_ball The ball invovled in the contact
     */
    void onDribblerBallEndContact(PhysicsRobot* physics_robot, PhysicsBall* physics_ball);

    /**
     * Helper functions that check if the current pointer to the physics_robot is valid
     * before calling the given function. If the physics_robot is invalid, a warning is
     * logged and a default value is returned.
     *
     * @param func The function to perform on the physics robot
     */
    void checkValidAndExecuteVoid(
        std::function<void(std::shared_ptr<PhysicsRobot>)> func);
    float checkValidAndReturnFloat(
        std::function<float(std::shared_ptr<PhysicsRobot>)> func);
    unsigned int checkValidAndReturnUint(
        std::function<unsigned int(std::shared_ptr<PhysicsRobot>)> func);

    /**
     * Applies force to the physics ball to simulate it being dribbled by the
     * physics robot.
     *
     * @param physics_robot The robot that should dribble the ball
     * @param physics_ball The ball to be dribbled
     */
    void applyDribblerForce(PhysicsRobot* physics_robot, PhysicsBall* physics_ball);

    std::weak_ptr<PhysicsRobot> physics_robot;
    std::optional<float> autokick_speed_m_per_s;
    std::optional<float> autochip_distance_m;
    uint32_t dribbler_rpm;

    typedef struct DribblerBall_t
    {
        PhysicsBall* ball;
        // We keep track of whether or not the ball can be controlled
        // in any way by the robot. This includes kicking, chipping,
        // and dribbling. This extra information is used to
        // prevent edge cases like the ball getting kicked/chipped
        // multiple times, and to prevent the dribbler from affecting
        // kicking
        bool can_be_controlled;
    } DribblerBall;

    std::optional<DribblerBall> ball_in_dribbler_area;

    std::unique_ptr<PrimitiveManager, FirmwarePrimitiveManagerDeleter> primitive_manager;

    // How much the dribbler damps the ball when they collide. Each component
    // of the damping can be changed separately so we have the flexibility to tune
    // this behavior to match real life. These values have been manually tuned
    // such that the robots are able to hit one-time shots in simulation with
    // sufficient accuracy.
    static constexpr double DRIBBLER_HEAD_ON_DAMPING       = 0.7;
    static constexpr double DRIBBLER_PERPENDICULAR_DAMPING = 0.61;
    // A value in the range [0, 1] that indicates how much momentum is conserved when the
    // ball is kicked. Higher values will cause the ball to be kicked with an even greater
    // velocity if it had an initial non-zero velocity when being kicked.
    // This value is a very rough estimate of real-world behaviour, so that the ball
    // will be kicked slightly faster it it entered the kicker with some initial velocity.
    static constexpr double MOMENTUM_CONSERVED_DURING_KICK = 0.1;
};
