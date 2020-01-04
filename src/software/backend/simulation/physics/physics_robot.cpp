#include "software/backend/simulation/physics/physics_robot.h"

#include "shared/constants.h"
#include "software/backend/simulation/physics/box2d_util.h"

PhysicsRobot::PhysicsRobot(std::shared_ptr<b2World> world, const Robot& robot)
    : robot_id(robot.id())
{
    // createRobotPhysicsBody must be called before the setup functions, so that the
    // b2Body is instantiated before fixtures are added to it.
    createRobotPhysicsBody(world, robot);
    setupRobotPhysicsBody(robot);
    setupChickerPhysicsBody(robot);
}

PhysicsRobot::~PhysicsRobot()
{
    // Examples for removing bodies safely from
    // https://www.iforce2d.net/b2dtut/removing-bodies
    b2World* robot_body_world = robot_body->GetWorld();
    if (bodyExistsInWorld(robot_body, robot_body_world))
    {
        robot_body_world->DestroyBody(robot_body);
    }
}

void PhysicsRobot::createRobotPhysicsBody(std::shared_ptr<b2World> world,
                                          const Robot& robot)
{
    // All the BodyDef must be defined before the body is created.
    // Changes made after aren't reflected
    robot_body_def.type = b2_dynamicBody;
    robot_body_def.position.Set(robot.position().x(), robot.position().y());
    robot_body_def.linearVelocity.Set(robot.velocity().x(), robot.velocity().y());
    robot_body_def.angle           = robot.orientation().toRadians();
    robot_body_def.angularVelocity = robot.angularVelocity().toRadians();

    robot_body = world->CreateBody(&robot_body_def);
}

void PhysicsRobot::setupRobotPhysicsBody(const Robot& robot)
{
    const unsigned int num_robot_body_vertices = b2_maxPolygonVertices;
    b2Vec2 robot_body_vertices[num_robot_body_vertices];
    auto vertices = getRobotBodyVertices(robot, num_robot_body_vertices);

    for (unsigned int i = 0; i < num_robot_body_vertices; i++)
    {
        robot_body_vertices[i] = createVec2(vertices.at(i));
    }

    robot_body_shape.Set(robot_body_vertices, num_robot_body_vertices);
    robot_fixture_def.shape = &robot_body_shape;

    robot_fixture_def.density = 1.0;
    // This is a somewhat arbitrary value. Collisions with robots are not perfectly
    // elastic. However because this is an "ideal" simulation and we generally don't care
    // about the exact behaviour of collisions, getting this value to perfectly match
    // reality isn't too important.
    robot_fixture_def.restitution = 0.5;
    robot_fixture_def.friction    = 0.0;

    robot_body->CreateFixture(&robot_fixture_def);
}

void PhysicsRobot::setupChickerPhysicsBody(const Robot& robot)
{
    // Create a very thin rectangle where the dribbler and chicker are on the
    // flat face of the robot
    robot_chicker_shape.SetAsBox(
        0.001, DRIBBLER_WIDTH_METERS / 2.0,
        createVec2(Vector::createFromAngle(robot.orientation())
                       .normalize(DIST_TO_FRONT_OF_ROBOT_METERS)),
        robot.orientation().toRadians());
    robot_chicker_def.shape = &robot_chicker_shape;

    // We don't want this shape to contribute to the mass of the overall robot, so density
    // is 0
    robot_chicker_def.density = 0.0;
    // restitution and friction are somewhat arbitrary values. Collisions with the chicker
    // are generally very damped, and friction along the dribbler is high since the
    // dribbler is designed to have high friction with the ball. However because this is
    // an "ideal" simulation and we generally don't care about the exact behaviour of the
    // ball colliding or rolling along the chicker / dribbler, getting these values to
    // perfectly match reality isn't too important.
    robot_fixture_def.restitution = 0.1;
    robot_fixture_def.friction    = 1.0;

    robot_body->CreateFixture(&robot_chicker_def);
}

std::vector<Point> PhysicsRobot::getRobotBodyVertices(const Robot& robot,
                                                      unsigned int num_vertices)
{
    // To create a polygon that approximates the shape of a robot, we find
    // one of the points at the edge of the flat face at the front of the robot,
    // and sweep around the back of the robot evenly distributing vertices until
    // we reach the other edge of the flat face.

    Angle angle_to_edge_of_flat_face_relative_to_robot_orientation =
        Vector(DIST_TO_FRONT_OF_ROBOT_METERS, FRONT_OF_ROBOT_WIDTH_METERS / 2.0)
            .orientation();
    Angle global_angle_to_edge_of_flat_face =
        angle_to_edge_of_flat_face_relative_to_robot_orientation + robot.orientation();

    Angle angle_to_sweep_across =
        Angle::full() - (2 * angle_to_edge_of_flat_face_relative_to_robot_orientation);
    Angle angle_increment = angle_to_sweep_across / (num_vertices - 1);

    std::vector<Point> robot_body_vertex_points;
    for (unsigned int i = 0; i < num_vertices; i++)
    {
        Angle angle_to_vertex = global_angle_to_edge_of_flat_face + (i * angle_increment);
        // Shapes are defined relative to the body they are added to, so we do everything
        // relative to the centre of the robot body, which is treated as (0, 0)
        Vector vector_from_body_to_vertex =
            Vector::createFromAngle(angle_to_vertex).normalize(ROBOT_MAX_RADIUS_METERS);
        Point vertex = Point(0, 0) + vector_from_body_to_vertex;
        robot_body_vertex_points.emplace_back(vertex);
    }

    return robot_body_vertex_points;
}

Robot PhysicsRobot::getRobotWithTimestamp(const Timestamp& timestamp) const
{
    Point position(robot_body->GetPosition().x, robot_body->GetPosition().y);
    Vector velocity(robot_body->GetLinearVelocity().x, robot_body->GetLinearVelocity().y);
    Angle orientation = Angle::fromRadians(robot_body->GetAngle());
    Angle angular_velocity =
        AngularVelocity::fromRadians(robot_body->GetAngularVelocity());

    Robot robot =
        Robot(robot_id, position, velocity, orientation, angular_velocity, timestamp);
    return robot;
}
