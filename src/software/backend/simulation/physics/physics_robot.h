#pragma once

#include <Box2D/Box2D.h>

#include "shared/constants.h"
#include "software/backend/simulation/physics/physics_ball.h"
#include "software/new_geom/point.h"
#include "software/time/timestamp.h"
#include "software/world/robot.h"

/**
 * This class represent a Robot in a Box2D physics simulation. It provides a convenient
 * way for us to abstract the robot and convert to our own Robot alss when data is needed.
 * This class only deals with physics and physics-related interactions, and does NOT
 * include any logic for robot behavior.
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

    /**
     * Creates a new PhysicsRobot given a Box2D world and a Robot object. A Box2D body
     * will be automatically added to the Box2D world and updated during world update
     * steps.
     *
     * @param world A shared_ptr to a Box2D World
     * @param robot The Robot to be created in the Box2D world
     * @param mass_kg The mass of the robot in kg
     */
    explicit PhysicsRobot(std::shared_ptr<b2World> world, const Robot& robot,
                          double mass_kg);

    PhysicsRobot() = delete;

    // Delete the copy and assignment operators because copying this class causes
    // issues with the b2World and how it tracks bodies in the world, because as objects
    // are copied and destroyed, they will destroy the bodies in the b2World as well
    PhysicsRobot& operator=(const PhysicsRobot&) = delete;
    PhysicsRobot(const PhysicsRobot&)            = delete;

    /**
     * Destroys the PhysicsRobot object and removes any corresponding bodies
     * from the physics world if the robot is part of one.
     */
    ~PhysicsRobot();

    /**
     * Adds the given function to this PhysicsRobot's list of dribbler-ball contact
     * callbacks. These callbacks will be called during every physics step, for the
     * duration of the contact.
     *
     * @param callback The function to register
     */
    void registerDribblerBallContactCallback(
        std::function<void(PhysicsRobot*, PhysicsBall*)> callback);

    /**
     * Adds the given function to this PhysicsRobot's list of dribbler-ball contact
     * callbacks. These callbacks will be called once at the start of the contact.
     *
     * @param callback The function to register
     */
    void registerDribblerBallStartContactCallback(
        std::function<void(PhysicsRobot*, PhysicsBall*)> callback);

    /**
     * Adds the given function to this PhysicsRobot's list of dribbler-ball contact
     * callbacks. These callbacks will be called once at the end of the contact.
     *
     * @param callback The function to register
     */
    void registerDribblerBallEndContactCallback(
        std::function<void(PhysicsRobot*, PhysicsBall*)> callback);

    /**
     * Adds the given function to this PhysicsRobot's list of chicker-ball contact
     * callbacks. These callbacks will be called once at the start of the contact.
     *
     * @param callback The function to register
     */
    void registerChickerBallStartContactCallback(
        std::function<void(PhysicsRobot*, PhysicsBall*)> callback);

    /**
     * Returns a list of contact callbacks for this class
     *
     * @return a list of contact callbacks for this class
     */
    std::vector<std::function<void(PhysicsRobot*, PhysicsBall*)>>
    getDribblerBallContactCallbacks() const;
    std::vector<std::function<void(PhysicsRobot*, PhysicsBall*)>>
    getDribblerBallStartContactCallbacks() const;
    std::vector<std::function<void(PhysicsRobot*, PhysicsBall*)>>
    getDribblerBallEndContactCallbacks() const;
    std::vector<std::function<void(PhysicsRobot*, PhysicsBall*)>>
    getChickerBallStartContactCallbacks() const;

    /**
     * Returns a Robot object representing the current state of the robot object in the
     * simulated Box2D world the robot was created in. The timestamp is provided as a
     * parameter so that the caller can control the timestamp of the data being returned,
     * since the caller will have context about the Box2D world and simulation time step,
     * and can synchronize the Robot timestamp with other objects.
     *
     * @param timestamp The timestamp for the returned Robot to have
     *
     * @return A Robot object representing the current state of the robot object in the
     * simulated Box2D world the robot was originally created in. The returned Robot
     * object will have the same timestamp as the one provided in the parameter
     */
    Robot getRobotWithTimestamp(const Timestamp& timestamp) const;

    /**
     * Returns the id of this physics robot
     *
     * @return the id of this physics robot
     */
    RobotId getRobotId() const;

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

   private:
    /**
     * Creates as many fixtures as necessary to represent the shape of the robot body, and
     * adds them to the robot's b2Body
     *
     * @param robot The robot to create fixtures for
     * @param total_chicker_depth The distance from the front face of the robot to the
     * back of the chicker, ie. how far inset into the front of the robot the chicker is
     * @param mass_kg The mass of the robot in kg
     */
    void setupRobotBodyFixtures(const Robot& robot, double total_chicker_depth,
                                double mass_kg);

    /**
     * Creates a fixture to represent the chicker of the robot. It is partially inset into
     * the front of the robot.
     *
     * @param robot The robot to create the fixture for
     * @param total_chicker_depth The distance from the front face of the robot to the
     * back of the chicker, ie. how far inset into the front of the robot the chicker is
     * @param chicker_thickness How thick the chicker fixture shape is
     */
    void setupChickerFixture(const Robot& robot, double total_chicker_depth,
                             double chicker_thickness);

    /**
     * Creates a fixture to represent the dribbler of the robot. It does not interact
     * physically with the world, it just serves as a way to detect when the ball is in
     * the dribbling area. The dribbler area fills the inset area at the front of the
     * robot.
     *
     * @param robot The robot to create the fixture for
     * @param dribbler_depth How far inset into the front of the robot the chicker is
     */
    void setupDribblerFixture(const Robot& robot, double dribbler_depth);

    /**
     * Together these functions return 3 polygons that together make up the shape of the
     * robot. It is broken up this way because Box2D can only have convex polygons. We
     * split the body into 3 parts:
     * - The main body, which is everything behind the chicker
     * - The front-left part, which is the bit that is to the front-left of the chicker,
     * partially enclosing it
     * - The front-right part, which is the bit that is to the front-right of the chicker,
     * partially enclosing it
     *
     * See the ASCII diagram below for a rough view of how the robot is created.
     * - The regions made with the '+' symbol are the front left and right bodies
     * - The region made with the 'x' symbol is the main body
     *
     *                                      xxxxxxxxxxxxxxxxxx
     *                                   x                      x
     *                                x                            x
     *                             x                                  x
     *                          x                                       X
     *                       x                                          x+++
     *                    x                                             x+  ++
     *                 x                                                x+   +
     *                 x                                                x+   +
     *                 x                                                x+++++
     *                 x                                                x|c|d|
     *                 x                                                x|h|r|
     *                 x                                                x|i|i|
     *                 x                                                x|c|b|
     *                 x                                                x|k|b|
     *                 x                                                x|e|l|
     *                 x                                                x|r|e|
     *                 x                                                x+++++
     *                 x                                                x+   +
     *                 x                                                x+   +
     *                    x                                             x+  ++
     *                       x                                          x+++
     *                          x                                       x
     *                             x                                  x
     *                                x                            x
     *                                   x                      x
     *                                      xxxxxxxxxxxxxxxxxx
     *
     *
     * @param robot The robot to create
     * @param total_chicker_depth The distance from the front face of the robot to the
     * back of the chicker, ie. how far inset into the front of the robot the chicker is
     *
     * @return A b2PolygonShape for the corresponding part of the robot body
     */
    b2PolygonShape* getMainRobotBodyShape(const Robot& robot, double total_chicker_depth);
    b2PolygonShape* getRobotBodyShapeFrontLeft(const Robot& robot,
                                               double total_chicker_depth);
    b2PolygonShape* getRobotBodyShapeFrontRight(const Robot& robot,
                                                double total_chicker_depth);

    /**
     * A helper function that returns the points that make up the front-left shape
     * for the robot body. The points that are returned assume the robot is at (0, 0)
     * and is facing the +x axis (aka has an orientation of 0). The points are returned
     * in counter-clockwise order.
     *
     * @param robot The robot to create
     * @param chicker_depth How far inset into the front of the robot the chicker is
     *
     * @return The points that make up the front-left shape for the robot body
     */
    std::vector<Point> getRobotFrontLeftShapePoints(const Robot& robot,
                                                    double chicker_depth);

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
    b2Body* robot_body;

    RobotId robot_id;

    std::vector<std::function<void(PhysicsRobot*, PhysicsBall*)>>
        dribbler_ball_contact_callbacks;
    std::vector<std::function<void(PhysicsRobot*, PhysicsBall*)>>
        dribbler_ball_start_contact_callbacks;
    std::vector<std::function<void(PhysicsRobot*, PhysicsBall*)>>
        dribbler_ball_end_contact_callbacks;
    std::vector<std::function<void(PhysicsRobot*, PhysicsBall*)>>
        chicker_ball_contact_callbacks;
};
