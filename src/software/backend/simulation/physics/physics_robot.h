#pragma once

#include <Box2D/Box2D.h>

#include "software/new_geom/point.h"
#include "software/time/timestamp.h"
#include "software/world/robot.h"

// TODO: comment
class PhysicsRobot
{
   public:
    /**
     * Creates a new PhysicsRobot given a Box2D world and a Robot object. A Box2D body
     * will be automatically added to the Box2D world and updated during world update
     * steps.
     *
     * @param world A shared_ptr to a Box2D World
     * @param robot The Robot to be created in the Box2D world
     * @param mass_kg The mass of the robot in kg
     */
    explicit PhysicsRobot(std::shared_ptr<b2World> world, const Robot& robot, double mass_kg);

    PhysicsRobot() = delete;

    // Delete the copy and assignment operators because copying this class causes
    // issues with the b2World and how it tracks bodies in the world, because as objects
    // are copied and destroyed, they will destroy the bodies in the b2World as well
    PhysicsRobot& operator=(const PhysicsRobot&) = delete;
    PhysicsRobot(const PhysicsRobot&)            = delete;

    /**
     * Destroys the PhysicsRobot object and removes any corresponding bodies
     * from the physics world if the robot is part of one
     */
    ~PhysicsRobot();

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
     * Returns the mass of the robot in kg
     *
     * @return the mass of the robot in kg
     */
    double getMassKg() const;

    // TODO: test
    /**
     * Sets the SimulatorRobot that this PhysicsRobot is associated with
     *
     * @param simulator_robot A pointer to the SimulatorRobot that this PhysicsRobot
     * is associated with
     */
    void setSimulatorRobot(SimulatorRobot* simulator_robot);

    /**
     * Returns a pointer to the SimulatorRobot this PhysicsRobot is associated with
     *
     * @return a pointer to the SimulatorRobot this PhysicsRobot is associated with
     */
    SimulatorRobot* getSimulatorRobot() const;

   private:
    /**
     * Creates as many fixtures as necessary to represent the shape of the robot body, and
     * adds them to the robot's b2Body
     *
     * @param robot The robot to create fixtures for
     * @param chicker_depth How far inset into the front of the robot the chicker is
     * @param mass_kg The mass of the robot in kg
     */
    void setupRobotBodyFixtures(const Robot& robot, double chicker_depth, double mass_kg);

    /**
     * Creates a fixture to represent the chicker of the robot. It is partially inset into
     * the front of the robot.
     *
     * @param robot The robot to create the fixture for
     * @param chicker_depth How far inset into the front of the robot the chicker is
     */
    void setupChickerFixture(const Robot& robot, double chicker_depth);

    /**
     * Creates a fixture to represent the dribbler of the robot. It does not interact
     * physically with the world, it just serves as a way to detect when the ball is in
     * the dribbling area. The dribbler area fills the inset area at the front of the
     * robot.
     *
     * @param robot The robot to create the fixture for
     * @param chicker_depth How far inset into the front of the robot the chicker is
     */
    void setupDribblerFixture(const Robot& robot, double chicker_depth);

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
     * @param chicker_depth How far inset into the front of the robot the chicker is
     *
     * @return A b2PolygonShape for the corresponding part of the robot body
     */
    b2PolygonShape* getMainRobotBodyShape(const Robot& robot, double chicker_depth);
    b2PolygonShape* getRobotBodyShapeFrontLeft(const Robot& robot, double chicker_depth);
    b2PolygonShape* getRobotBodyShapeFrontRight(const Robot& robot, double chicker_depth);

    /**
     * A helper function that returns the points that make up the front-left shape
     * for the robot body. The points that are returned assume the robot is at (0, 0)
     * and is facing the +x axis (aka has an orientation of 0)
     *
     * @param robot The robot to create
     * @param chicker_depth How far inset into the front of the robot the chicker is
     *
     * @return The points that make up the front-left shape for the robot body
     */
    std::vector<Point> getRobotFrontLeftShapePoints(const Robot& robot,
                                                    double chicker_depth);

    // See https://box2d.org/manual.pdf chapters 6 and 7 more information on Shapes,
    // Bodies, and Fixtures
    b2Body* robot_body;
    RobotId robot_id;

    // The SimulatorRobot associated with this PhysicsRobot, if any
    SimulatorRobot* simulator_robot;
};
