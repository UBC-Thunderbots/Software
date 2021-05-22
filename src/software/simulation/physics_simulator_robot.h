#pragma once

#include <cinttypes>
#include <memory>

#include "software/logger/logger.h"
#include "software/simulation/force_wheel_simulator_robot.h"
#include "software/simulation/physics/physics_ball.h"
#include "software/simulation/physics/physics_robot.h"

/**
 * The PhysicsSimulatorRobot class acts as a wrapper for a PhysicsRobot that deals with
 * more logic-focused elements for simulation, such as whether or not autokick is enabled.
 *
 * All members of this class are intentionally protected or private to force this class
 * to only be controlled by the SimulatorRobotSingleton.
 */
class PhysicsSimulatorRobot : public ForceWheelSimulatorRobot
{
   public:
    /**
     * Create a new PhysicsSimulatorRobot given a PhysicsRobot
     *
     * @param physics_robot the PhysicsRobot to simulate and control
     */
    explicit PhysicsSimulatorRobot(std::weak_ptr<PhysicsRobot> physics_robot);
    explicit PhysicsSimulatorRobot() = delete;

    unsigned int getRobotId() override;

    /**
     * Clears balls tracked as being in dribbler area
     */
    void clearBallInDribblerArea();

   protected:
    float getPositionX() override;

    float getPositionY() override;

    float getOrientation() override;

    float getVelocityX() override;

    float getVelocityY() override;

    float getVelocityAngular() override;

    float getBatteryVoltage() override;

    void kick(float speed_m_per_s) override;

    void chip(float distance_m) override;

    void enableAutokick(float speed_m_per_s) override;

    void enableAutochip(float distance_m) override;

    void disableAutokick() override;

    void disableAutochip() override;

    bool isAutokickEnabled() override;

    bool isAutochipEnabled() override;

    void setDribblerSpeed(uint32_t rpm) override;

    void dribblerCoast() override;

    unsigned int getDribblerTemperatureDegC() override;

    void applyWheelForceFrontLeft(float force_in_newtons) override;
    void applyWheelForceBackLeft(float force_in_newtons) override;
    void applyWheelForceBackRight(float force_in_newtons) override;
    void applyWheelForceFrontRight(float force_in_newtons) override;

    float getMotorSpeedFrontLeft() override;
    float getMotorSpeedBackLeft() override;
    float getMotorSpeedBackRight() override;
    float getMotorSpeedFrontRight() override;

    void coastMotorBackLeft() override;
    void coastMotorBackRight() override;
    void coastMotorFrontLeft() override;
    void coastMotorFrontRight() override;

    void brakeMotorBackLeft() override;
    void brakeMotorBackRight() override;
    void brakeMotorFrontLeft() override;
    void brakeMotorFrontRight() override;

    void startNewPrimitive(std::shared_ptr<FirmwareWorld_t> firmware_world,
                           const TbotsProto_Primitive& primitive_msg) override;

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
    template <class T>
    T checkValidAndExecute(std::function<T(std::shared_ptr<PhysicsRobot>)> func)
    {
        if (auto robot = physics_robot.lock())
        {
            return func(robot);
        }
        else
        {
            LOG(WARNING) << "PhysicsSimulatorRobot being used with invalid PhysicsRobot"
                         << std::endl;
            return static_cast<T>(0);
        }
    }

    /**
     * Applies force to the physics ball to simulate it being dribbled by the
     * physics robot.
     *
     * @param physics_robot The robot that should dribble the ball
     * @param physics_ball The ball to be dribbled
     */
    void applyDribblerForce(PhysicsRobot* physics_robot, PhysicsBall* physics_ball);

    std::weak_ptr<PhysicsRobot> physics_robot;
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
