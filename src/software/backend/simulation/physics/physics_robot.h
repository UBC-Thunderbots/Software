#pragma once

#include <Box2D/Box2D.h>

#include "software/new_geom/point.h"
#include "software/time/timestamp.h"
#include "software/world/robot.h"

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
     */
    explicit PhysicsRobot(std::shared_ptr<b2World> world, const Robot& robot);

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

    RobotId getRobotId() const;

   private:
    /**
     * A helper function that creates the robot body object in the physics world
     *
     * @param world A shared_ptr to a Box2D World
     * @param robot The Robot being created in the Box2D world
     */
    void createRobotPhysicsBody(std::shared_ptr<b2World> world, const Robot& robot);

    /**
     * A helper function that defines and adds fixtures to the robot physics object
     * that represent the robot body
     *
     * @param robot The Robot being created in the Box2D world
     */
    void setupRobotPhysicsBody(const Robot& robot);

    /**
     * A helper function that defines and adds fixtures to the robot physics object
     * that represent the robot's chicker
     *
     * @param robot The Robot being created in the Box2D world
     */
    void setupChickerPhysicsBody(const Robot& robot);

    /**
     * Returns a list of points that represent the vertices of a polygon approximating
     * the shape of the robot body. The points are relative to the center of the robot,
     * which is assumed to be at (0, 0)
     *
     * @param robot The robot
     * @param num_vertices How many vertices to create
     *
     * @return A list of vertices that make up a polygon approximating the shape of the
     * robot body
     */
    std::vector<Point> getRobotBodyVertices(const Robot& robot,
                                            unsigned int num_vertices);

    // See https://box2d.org/manual.pdf chapters 6 and 7 more information on Shapes,
    // Bodies, and Fixtures
    b2PolygonShape robot_body_shape;
    b2FixtureDef robot_fixture_def;
    b2PolygonShape robot_chicker_shape;
    b2FixtureDef robot_chicker_def;
    b2BodyDef robot_body_def;
    b2Body* robot_body;

    RobotId robot_id;
};
