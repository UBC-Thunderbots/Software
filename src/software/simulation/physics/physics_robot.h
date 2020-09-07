#pragma once

#include <Box2D/Box2D.h>

#include <functional>

#include "software/geom/angle.h"
#include "software/geom/angular_velocity.h"
#include "software/geom/point.h"
#include "software/geom/vector.h"
#include "software/multithreading/thread_safe_buffer.h"
#include "software/simulation/physics/physics_ball.h"
#include "software/world/robot_state.h"

class PhysicsWorld;

/**
 * This class represent a Robot in a Box2D physics simulation. It provides a convenient
 * way for us to abstract the robot and convert to our own Robot alss when data is needed.
 * This class only deals with physics and physics-related interactions, and does NOT
 * include any logic for robot behavior.
 *
 * This class returns floats in some cases because Box2D works with floats
 */
class PhysicsRobot
{
   public:
    // The depth of the dribbling area at the front of the robot.
    // We assume the ball can be dribbled as long as it is anywhere within this small
    // area.
    static double const dribbler_depth;
    // The thickness of the chicker fixture shape.
    static double const chicker_thickness;
    static double const total_chicker_depth;

    friend class PhysicsWorld;

    /**
     * Creates a new PhysicsRobot given a Box2D world and a Robot object. A Box2D body
     * will be automatically added to the Box2D world and updated during world update
     * steps.
     *
     * @param id The id of the robot
     * @param world A shared_ptr to a Box2D World
     * @param robot_state The initial robot state
     * @param mass_kg The mass of the robot in kg
     */
    explicit PhysicsRobot(const RobotId id, std::shared_ptr<b2World> world,
                          const RobotState &robot_state, const double mass_kg);

    PhysicsRobot() = delete;

    // Delete the copy and assignment operators because copying this class causes
    // issues with the b2World and how it tracks bodies in the world, because as objects
    // are copied and destroyed, they will destroy the bodies in the b2World as well
    PhysicsRobot &operator=(const PhysicsRobot &) = delete;
    PhysicsRobot(const PhysicsRobot &)            = delete;

    /**
     * Destroys the PhysicsRobot object and removes any corresponding bodies
     * from the physics world if the robot is part of one.
     */
    ~PhysicsRobot();

    /**
     * Returns the id of this physics robot
     *
     * @return the id of this physics robot
     */
    RobotId getRobotId() const;

    /**
     * Adds the given function to this PhysicsRobot's list of dribbler-ball contact
     * callbacks. These callbacks will be called during every physics step, for the
     * duration of the contact.
     *
     * @param callback The function to register
     */
    void registerDribblerBallContactCallback(
        std::function<void(PhysicsRobot *, PhysicsBall *)> callback);

    /**
     * Adds the given function to this PhysicsRobot's list of dribbler-ball contact
     * callbacks. These callbacks will be called once at the start of the contact.
     *
     * @param callback The function to register
     */
    void registerDribblerBallStartContactCallback(
        std::function<void(PhysicsRobot *, PhysicsBall *)> callback);

    /**
     * Adds the given function to this PhysicsRobot's list of dribbler-ball contact
     * callbacks. These callbacks will be called once at the end of the contact.
     *
     * @param callback The function to register
     */
    void registerDribblerBallEndContactCallback(
        std::function<void(PhysicsRobot *, PhysicsBall *)> callback);

    /**
     * Adds the given function to this PhysicsRobot's list of chicker-ball contact
     * callbacks. These callbacks will be called once at the start of the contact.
     *
     * @param callback The function to register
     */
    void registerChickerBallStartContactCallback(
        std::function<void(PhysicsRobot *, PhysicsBall *)> callback);

    /**
     * Returns a list of contact callbacks for this class
     *
     * @return a list of contact callbacks for this class
     */
    std::vector<std::function<void(PhysicsRobot *, PhysicsBall *)>>
    getDribblerBallContactCallbacks() const;
    std::vector<std::function<void(PhysicsRobot *, PhysicsBall *)>>
    getDribblerBallStartContactCallbacks() const;
    std::vector<std::function<void(PhysicsRobot *, PhysicsBall *)>>
    getDribblerBallEndContactCallbacks() const;
    std::vector<std::function<void(PhysicsRobot *, PhysicsBall *)>>
    getChickerBallStartContactCallbacks() const;

    /**
     * Returns the current robot state
     *
     * @return the current robot state
     */
    RobotState getRobotState() const;

    /**
     * Returns the current position of the robot, in global field coordinates, in meters
     *
     * @return the current position of the robot, in global field coordinates, in meters
     */
    Point position() const;

    /**
     * Returns the current velocity of the robot, in global field coordinates, in m/s
     *
     * @return the current velocity of the robot, in global field coordinates, in m/s
     */
    Vector velocity() const;

    /**
     * Returns the current orientation of the robot, in global field coordinates
     *
     * @return the current orientation of the robot, in global field coordinates
     */
    Angle orientation() const;

    /**
     * Returns the current angular velocity of the robot, in global field coordinates
     *
     * @return the current angular velocity of the robot, in global field coordinates
     */
    AngularVelocity angularVelocity() const;

    /**
     * Applies the given force to the wheel. Positive force spins the wheel
     * counter-clockwise, as viewed from inside the robot looking out
     * (ie. induces positive angular velocity and rotation to the robot).
     * Negative force does the opposite.
     *
     * @param force_in_newtons the force to apply to the wheel
     */
    void applyWheelForceFrontLeft(double force_in_newtons);
    void applyWheelForceBackLeft(double force_in_newtons);
    void applyWheelForceBackRight(double force_in_newtons);
    void applyWheelForceFrontRight(double force_in_newtons);

    /**
     * Gets the motor speed for the wheel, in RPM
     */
    float getMotorSpeedFrontLeft();
    float getMotorSpeedBackLeft();
    float getMotorSpeedBackRight();
    float getMotorSpeedFrontRight();

    /**
     * Sets the motor to brake (act against the current direction of rotation)
     */
    void brakeMotorBackLeft();
    void brakeMotorBackRight();
    void brakeMotorFrontLeft();
    void brakeMotorFrontRight();

    /**
     * Sets the position of the PhysicsRobot to the given position. The robot
     * will maintain its orientation, but will have its linear and angular
     * velocity set to zero.
     *
     * @param position The new position of the robot
     */
    void setPosition(const Point &position);

   protected:
    /**
     * This functions runs any operations this PhysicsRobot wants to perform
     * after a physics step has happened. We assume this function will be called
     * by the PhysicsWorld at the correct time. This exists so that this class
     * doesn't accidentally make a change to the Box2D world in the middle of
     * a physics step, which causes the system to crash.
     */
    void runPostPhysicsStep();

   private:
    /**
     * Creates as many fixtures as necessary to represent the body shape of the given
     * robot and add them to this class' b2Body
     *
     * @param robot_state The robot to create fixtures for
     * @param total_chicker_depth The distance from the front face of the robot to the
     * back of the chicker, ie. how far inset into the front of the robot the chicker is
     * @param mass_kg The mass of the robot in kg
     */
    void setupRobotBodyFixtures(const RobotState &robot_state, double total_chicker_depth,
                                double mass_kg);

    /**
     * Creates a robot body fixture with the given shape and density and adds it to this
     * class' b2Body
     *
     * @param shape The shape to make the fixture with
     * @param density The density of the shape
     */
    void setupRobotBodyFixture(const b2PolygonShape *shape, const float density);

    /**
     * Creates a fixture to represent the chicker of the robot. It is partially inset into
     * the front of the robot.
     *
     * @param robot_state The robot to create the fixture for
     * @param total_chicker_depth The distance from the front face of the robot to the
     * back of the chicker, ie. how far inset into the front of the robot the chicker is
     * @param chicker_thickness How thick the chicker fixture shape is
     */
    void setupChickerFixture(const RobotState &robot_state, double total_chicker_depth,
                             double chicker_thickness);

    /**
     * Creates a fixture to represent the dribbler of the robot. It does not interact
     * physically with the world, it just serves as a way to detect when the ball is in
     * the dribbling area. The dribbler area fills the inset area at the front of the
     * robot.
     *
     * @param robot_state The robot to create the fixture for
     * @param dribbler_depth How far inset into the front of the robot the chicker is
     */
    void setupDribblerFixture(const RobotState &robot_state, double dribbler_depth);

    /**
     * A helper function that applies force to the robot body as if there was a wheel
     * at the given angle, relative to the front of the robot
     *
     * @param angle_to_wheel The angle to the wheel axis, relative to the front of the
     * robot
     * @param force_in_newtons The force to apply
     */
    void applyWheelForceAtAngle(Angle angle_to_wheel, double force_in_newtons);

    /**
     * Returns the motor speeds for all motors on the robot. Units are in rpm
     *
     * @return the motor speeds for all motors on the robot. Units are in rpm
     */
    std::array<float, 4> getMotorSpeeds() const;

    /**
     * Returns how much force should be applied to a wheel when the motor is braking.
     *
     * @param motor_speed The current speed of the motor, in rpm
     *
     * @return How much force (in Newtons) to apply to the wheel to simulate a braking
     * effect
     */
    float getMotorBrakeForce(float motor_speed) const;

    // See https://box2d.org/manual.pdf chapters 6 and 7 more information on Shapes,
    // Bodies, and Fixtures
    b2Body *robot_body;

    RobotId robot_id;

    std::vector<std::function<void(PhysicsRobot *, PhysicsBall *)>>
        dribbler_ball_contact_callbacks;
    std::vector<std::function<void(PhysicsRobot *, PhysicsBall *)>>
        dribbler_ball_start_contact_callbacks;
    std::vector<std::function<void(PhysicsRobot *, PhysicsBall *)>>
        dribbler_ball_end_contact_callbacks;
    std::vector<std::function<void(PhysicsRobot *, PhysicsBall *)>>
        chicker_ball_contact_callbacks;

    std::queue<std::function<void()>> post_physics_step_functions;

    // This is a somewhat arbitrary value for damping. We keep it relatively low
    // so that robots still coast a ways before stopping, but non-zero so that robots
    // do come to a halt if no force is applied.
    // Damping is roughly calculated as v_new = v * exp(-damping * t)
    // https://gamedev.stackexchange.com/questions/160047/what-does-lineardamping-mean-in-box2d
    // TODO: These values are currently increased in order for the bang-bang controller
    // simulated in the robot firmware to behave reasonably. These values should be
    // changed back to something around 0.2
    // https://github.com/UBC-Thunderbots/Software/issues/1187
    const double robot_linear_damping  = 2.0;
    const double robot_angular_damping = 2.0;

    // This is a somewhat arbitrary value. Collisions with robots are not perfectly
    // elastic. However because this is an "ideal" simulation and we generally don't care
    // about the exact behaviour of collisions, getting this value to perfectly match
    // reality isn't too important.
    const double robot_body_restitution = 0.5;
    const double robot_body_friction    = 0.0;

    // The dribbler has no mass in simulation. Mass is already accounted for by the robot
    // body
    const double robot_dribbler_density = 0.0;
    // We assume the ContactListener for the world will disable collisions with the
    // dribbler so the restitution and friction don't matter
    const double robot_dribbler_restitution = 0.0;
    const double robot_dribbler_friction    = 0.0;

    // We want perfect damping to help us keep the ball when dribbling
    const double robot_chicker_restitution = 0.0;
    // We want lots of friction for when the ball is being dribbled so it stays controlled
    const double robot_chicker_friction = 1.0;
    // The chicker has no mass in simulation. Mass is already accounted for by the robot
    // body
    const double robot_chicker_density = 0.0;
};
