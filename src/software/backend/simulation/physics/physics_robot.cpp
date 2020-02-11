#include "software/backend/simulation/physics/physics_robot.h"
#include "software/backend/simulation/physics/physics_object_user_data.h"

#include <algorithm>
#include <numeric>

#include "shared/constants.h"
#include "software/backend/simulation/physics/box2d_util.h"
extern "C" {
#include "firmware/main/shared/physics.h"
}

PhysicsRobot::PhysicsRobot(std::shared_ptr<b2World> world, const Robot& robot, double mass_kg)
    : robot_id(robot.id())
{
    b2BodyDef robot_body_def;
    robot_body_def.type = b2_dynamicBody;
    robot_body_def.position.Set(robot.position().x(), robot.position().y());
    robot_body_def.linearVelocity.Set(robot.velocity().x(), robot.velocity().y());
    robot_body_def.angle           = robot.orientation().toRadians();
    robot_body_def.angularVelocity = robot.angularVelocity().toRadians();

    robot_body = world->CreateBody(&robot_body_def);

    // The depth of the inset area at the front of the robot. This is where the chicker is
    // in real life, so we create an inset since the ball makes contact slightly inside
    // the robot body. We are only allowed to cover a fraction of the ball, so we use this
    // to determine the depth of the inset
    double chicker_depth = BALL_MAX_RADIUS_METERS * 2 * MAX_FRACTION_OF_BALL_COVERED_BY_ROBOT;

    setupRobotBodyFixtures(robot, chicker_depth, mass_kg);
    setupDribblerFixture(robot, chicker_depth);
    setupChickerFixture(robot, chicker_depth);
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

void PhysicsRobot::setupRobotBodyFixtures(const Robot& robot, double chicker_depth, double mass_kg)
{
    b2FixtureDef robot_body_fixture_def;
    // This is a somewhat arbitrary value. Collisions with robots are not perfectly
    // elastic. However because this is an "ideal" simulation and we generally don't care
    // about the exact behaviour of collisions, getting this value to perfectly match
    // reality isn't too important.
    robot_body_fixture_def.restitution = 0.0; // TODO: change back to 0.5?
    robot_body_fixture_def.friction    = 0.0;
    robot_body_fixture_def.userData = new PhysicsObjectUserData({PhysicsObjectType::ROBOT_BODY, this});

    b2PolygonShape* main_body_shape = getMainRobotBodyShape(robot, chicker_depth);
    b2PolygonShape* front_left_body_shape =
        getRobotBodyShapeFrontLeft(robot, chicker_depth);
    b2PolygonShape* front_right_body_shape =
        getRobotBodyShapeFrontRight(robot, chicker_depth);

    auto body_shapes = {main_body_shape, front_left_body_shape, front_right_body_shape};
//    body_shapes = {main_body_shape};
    double total_shape_area = 0.0;
    for (const auto* shape : body_shapes)
    {
        total_shape_area += polygonArea(*shape);
    }
    robot_body_fixture_def.density = mass_kg / total_shape_area;

    for (const auto shape : body_shapes)
    {
        robot_body_fixture_def.shape = shape;
        robot_body->CreateFixture(&robot_body_fixture_def);
    }
}

void PhysicsRobot::setupDribblerFixture(const Robot& robot, double chicker_depth)
{
    b2FixtureDef robot_dribbler_fixture_def;
    // We assume collisions with the dribbler get disabled, so these values
    // don't matter
    robot_dribbler_fixture_def.restitution = 0.0;
    robot_dribbler_fixture_def.friction    = 0.0;
    // The dribbler has no mass in simulation. Mass is already accounted for by the robot
    // body
    robot_dribbler_fixture_def.density     = 0.0;
    // TODO: comment
    robot_dribbler_fixture_def.isSensor = false;
    robot_dribbler_fixture_def.userData = new PhysicsObjectUserData({PhysicsObjectType::ROBOT_DRIBBLER, this});

    const unsigned int num_vertices              = 4;
    b2Vec2 dribbler_shape_vertices[num_vertices] = {
        createVec2(Point(DIST_TO_FRONT_OF_ROBOT_METERS, DRIBBLER_WIDTH_METERS / 2.0)
                       .rotate(robot.orientation())),
        createVec2(Point(DIST_TO_FRONT_OF_ROBOT_METERS - chicker_depth,
                         DRIBBLER_WIDTH_METERS / 2.0)
                       .rotate(robot.orientation())),
        createVec2(Point(DIST_TO_FRONT_OF_ROBOT_METERS - chicker_depth,
                         -DRIBBLER_WIDTH_METERS / 2.0)
                       .rotate(robot.orientation())),
        createVec2(Point(DIST_TO_FRONT_OF_ROBOT_METERS, -DRIBBLER_WIDTH_METERS / 2.0)
                       .rotate(robot.orientation()))};
    b2PolygonShape* dribbler_shape = new b2PolygonShape();
    dribbler_shape->Set(dribbler_shape_vertices, num_vertices);
    robot_dribbler_fixture_def.shape = dribbler_shape;
    robot_body->CreateFixture(&robot_dribbler_fixture_def);
}

void PhysicsRobot::setupChickerFixture(const Robot& robot, double chicker_depth)
{
    b2FixtureDef robot_chicker_fixture_def;
    // We want perfect damping to help us keep the ball when dribbling
    robot_chicker_fixture_def.restitution = 0.0;
    // We want lots of friction for when the ball is being dribbled so it stays controlled
    robot_chicker_fixture_def.friction = 1.0;
    // The chicker has no mass in simulation. Mass is already accounted for by the robot
    // body
    robot_chicker_fixture_def.density = 0.0;
    robot_chicker_fixture_def.userData = new PhysicsObjectUserData({PhysicsObjectType::ROBOT_CHICKER, this});

    const unsigned int num_vertices             = 4;
    b2Vec2 chicker_shape_vertices[num_vertices] = {
        createVec2(Point(DIST_TO_FRONT_OF_ROBOT_METERS - chicker_depth / 2.0,
                         DRIBBLER_WIDTH_METERS / 2.0)
                       .rotate(robot.orientation())),
        createVec2(Point(DIST_TO_FRONT_OF_ROBOT_METERS - chicker_depth,
                         DRIBBLER_WIDTH_METERS / 2.0)
                       .rotate(robot.orientation())),
        createVec2(Point(DIST_TO_FRONT_OF_ROBOT_METERS - chicker_depth,
                         -DRIBBLER_WIDTH_METERS / 2.0)
                       .rotate(robot.orientation())),
        createVec2(Point(DIST_TO_FRONT_OF_ROBOT_METERS - chicker_depth / 2.0,
                         -DRIBBLER_WIDTH_METERS / 2.0)
                       .rotate(robot.orientation()))};
    b2PolygonShape* chicker_shape = new b2PolygonShape();
    chicker_shape->Set(chicker_shape_vertices, num_vertices);
    robot_chicker_fixture_def.shape = chicker_shape;
    robot_body->CreateFixture(&robot_chicker_fixture_def);
}

b2PolygonShape* PhysicsRobot::getMainRobotBodyShape(const Robot& robot,
                                                    double chicker_depth)
{
    const unsigned int num_shape_vertices = b2_maxPolygonVertices;
    b2Vec2 robot_body_vertices[num_shape_vertices];

    // Assuming the robot is at (0, 0) and facing the +x axis (aka has an orientation of
    // 0) First find the y-coordinate of the front-left edge of the body by solving x^2 +
    // y^2 = ROBOT_RADIUS^2
    double y = std::sqrt(std::pow(ROBOT_MAX_RADIUS_METERS, 2) -
                         std::pow(DIST_TO_FRONT_OF_ROBOT_METERS - chicker_depth, 2));

    Vector starting_vector(DIST_TO_FRONT_OF_ROBOT_METERS - chicker_depth, y);
    Angle starting_angle        = starting_vector.orientation();
    Angle angle_to_sweep_across = Angle::full() - (2 * starting_angle);
    Angle angle_increment       = angle_to_sweep_across / (num_shape_vertices - 1);
    for (unsigned int i = 0; i < num_shape_vertices; i++)
    {
        Angle angle_to_vertex = starting_angle + (i * angle_increment);
        Point vertex =
            Point(0, 0) +
            Vector::createFromAngle(angle_to_vertex).normalize(ROBOT_MAX_RADIUS_METERS);
        // The shape is added relative to the body, so we do not need to rotate these points
        // to match the robot's orientation
        robot_body_vertices[i] = createVec2(vertex);
    }

    b2PolygonShape* body_shape = new b2PolygonShape();
    body_shape->Set(robot_body_vertices, num_shape_vertices);
    return body_shape;
}

b2PolygonShape* PhysicsRobot::getRobotBodyShapeFrontLeft(const Robot& robot,
                                                         double chicker_depth)
{
    auto shape_points = getRobotFrontLeftShapePoints(robot, chicker_depth);

    // The shape is added relative to the body, so we do not need to rotate these points
    // to match the robot's orientation

    b2Vec2 vertices[shape_points.size()];
    for (unsigned int i = 0; i < shape_points.size(); i++)
    {
        vertices[i] = createVec2(shape_points.at(i));
    }

    b2PolygonShape* shape = new b2PolygonShape();
    shape->Set(vertices, shape_points.size());

    return shape;
}

b2PolygonShape* PhysicsRobot::getRobotBodyShapeFrontRight(const Robot& robot,
                                                          double chicker_depth)
{
    auto shape_points = getRobotFrontLeftShapePoints(robot, chicker_depth);

    // Mirror the points over the x-axis to get the points for the front-right shape
    std::transform(shape_points.begin(), shape_points.end(), shape_points.begin(),
                   [](const Point& p) { return Point(p.x(), -(p.y())); });

    // The shape is added relative to the body, so we do not need to rotate these points
    // to match the robot's orientation

    b2Vec2 vertices[shape_points.size()];
    for (unsigned int i = 0; i < shape_points.size(); i++)
    {
        vertices[i] = createVec2(shape_points.at(i));
    }

    b2PolygonShape* shape = new b2PolygonShape();
    shape->Set(vertices, shape_points.size());

    return shape;
}

std::vector<Point> PhysicsRobot::getRobotFrontLeftShapePoints(const Robot& robot,
                                                              double chicker_depth)
{
    // Assuming the robot is at (0, 0) and facing the +x axis (aka has an orientation of
    // 0) First find the y-coordinate of the front-left edge of the body by solving x^2 +
    // y^2 = ROBOT_RADIUS^2
    double y = std::sqrt(std::pow(ROBOT_MAX_RADIUS_METERS, 2) -
                         std::pow(DIST_TO_FRONT_OF_ROBOT_METERS - chicker_depth, 2));

    // Box2D requires polygon vertices are provided in counter-clockwise order
    std::vector<Point> vertices{
        Point(DIST_TO_FRONT_OF_ROBOT_METERS - chicker_depth, y),
        Point(DIST_TO_FRONT_OF_ROBOT_METERS - chicker_depth, DRIBBLER_WIDTH_METERS / 2.0),
        Point(DIST_TO_FRONT_OF_ROBOT_METERS, DRIBBLER_WIDTH_METERS / 2.0),
        Point(DIST_TO_FRONT_OF_ROBOT_METERS, FRONT_OF_ROBOT_WIDTH_METERS / 2.0)
    };

    return vertices;
}

void PhysicsRobot::registerDribblerBallContactCallback(std::function<void(PhysicsRobot *, PhysicsBall *)> callback) {
    dribbler_ball_contact_callbacks.emplace_back(callback);
}

void PhysicsRobot::registerDribblerBallStartContactCallback(std::function<void(PhysicsRobot *, PhysicsBall *)> callback) {
    dribbler_ball_start_contact_callbacks.emplace_back(callback);
}

void PhysicsRobot::registerDribblerBallEndContactCallback(std::function<void(PhysicsRobot *, PhysicsBall *)> callback) {
    dribbler_ball_end_contact_callbacks.emplace_back(callback);
}

void PhysicsRobot::registerChickerBallContactCallback(std::function<void(PhysicsRobot *, PhysicsBall *)> callback) {
    chicker_ball_contact_callbacks.emplace_back(callback);
}

std::vector<std::function<void(PhysicsRobot*, PhysicsBall*)>> PhysicsRobot::getDribblerBallContactCallbacks() const {
    return dribbler_ball_contact_callbacks;
}

std::vector<std::function<void(PhysicsRobot*, PhysicsBall*)>> PhysicsRobot::getDribblerBallStartContactCallbacks() const {
    return dribbler_ball_start_contact_callbacks;
}

std::vector<std::function<void(PhysicsRobot*, PhysicsBall*)>> PhysicsRobot::getDribblerBallEndContactCallbacks() const {
    return dribbler_ball_end_contact_callbacks;
}

std::vector<std::function<void(PhysicsRobot*, PhysicsBall*)>> PhysicsRobot::getChickerBallContactCallbacks() const {
   return chicker_ball_contact_callbacks;
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

RobotId PhysicsRobot::getRobotId() const
{
    return robot_id;
}

void PhysicsRobot::applyWheelForceFrontLeft(double force_in_newtons) {
    Angle angle_to_wheel = Angle::fromDegrees(ANGLE_TO_ROBOT_FRONT_WHEELS_DEG);
    applyWheelForceAtAngle(angle_to_wheel, force_in_newtons);
}

void PhysicsRobot::applyWheelForceBackLeft(double force_in_newtons) {
    Angle angle_to_wheel = Angle::fromDegrees(ANGLE_TO_ROBOT_BACK_WHEELS_DEG);
    applyWheelForceAtAngle(angle_to_wheel, force_in_newtons);
}

void PhysicsRobot::applyWheelForceBackRight(double force_in_newtons) {
    Angle angle_to_wheel = Angle::fromDegrees(-ANGLE_TO_ROBOT_BACK_WHEELS_DEG);
    applyWheelForceAtAngle(angle_to_wheel, force_in_newtons);
}

void PhysicsRobot::applyWheelForceFrontRight(double force_in_newtons) {
    Angle angle_to_wheel = Angle::fromDegrees(-ANGLE_TO_ROBOT_FRONT_WHEELS_DEG);
    applyWheelForceAtAngle(angle_to_wheel, force_in_newtons);
}

void PhysicsRobot::applyWheelForceAtAngle(Angle angle_to_wheel, double force_in_newtons) {
    // The center of the robot is always at (0, 0) in its own coordinate frame
    Point local_robot_position = Point(0, 0);
    Point local_force_point = local_robot_position + Vector::createFromAngle(angle_to_wheel).normalize(ROBOT_MAX_RADIUS_METERS);
    Vector local_force_vector = (local_force_point - local_robot_position).perpendicular();
    local_force_vector = local_force_vector.normalize(force_in_newtons);
    b2Vec2 force_point = robot_body->GetWorldPoint(createVec2(local_force_point));
    b2Vec2 force_vector = robot_body->GetWorldVector(createVec2(local_force_vector));
    robot_body->ApplyForce(force_vector, force_point, true);
}

std::array<float, 4> PhysicsRobot::getMotorSpeeds() const {
    float robot_local_speed[3] {
            robot_body->GetLocalVector(robot_body->GetLinearVelocity()).x,
            robot_body->GetLocalVector(robot_body->GetLinearVelocity()).y,
            robot_body->GetAngularVelocity()
    };
    float wheel_speeds[4] {0.0, 0.0, 0.0, 0.0};
    speed3_to_speed4(robot_local_speed, wheel_speeds);
    std::array<float, 4> motor_speeds = {0.0, 0.0, 0.0, 0.0};
    for(unsigned int i = 0; i < 4; i++) {
        motor_speeds[i] = wheel_speeds[i] / GEAR_RATIO;
    }
    return motor_speeds;
}

float PhysicsRobot::getMotorSpeedFrontLeft() {
    return getMotorSpeeds()[0];
}

float PhysicsRobot::getMotorSpeedBackLeft() {
    return getMotorSpeeds()[1];
}

float PhysicsRobot::getMotorSpeedBackRight() {
    return getMotorSpeeds()[2];
}

float PhysicsRobot::getMotorSpeedFrontRight() {
    return getMotorSpeeds()[3];
}

void PhysicsRobot::brakeMotorFrontLeft() {
    float motor_speed = getMotorSpeedFrontLeft();
    float wheel_force = getMotorBrakeForce(motor_speed);
    applyWheelForceFrontLeft(wheel_force);
}

void PhysicsRobot::brakeMotorBackLeft() {
    float motor_speed = getMotorSpeedBackLeft();
    float wheel_force = getMotorBrakeForce(motor_speed);
    applyWheelForceBackLeft(wheel_force);
}

void PhysicsRobot::brakeMotorBackRight() {
    float motor_speed = getMotorSpeedBackRight();
    float wheel_force = getMotorBrakeForce(motor_speed);
    applyWheelForceBackRight(wheel_force);
}

void PhysicsRobot::brakeMotorFrontRight() {
    float motor_speed = getMotorSpeedFrontRight();
    float wheel_force = getMotorBrakeForce(motor_speed);
    applyWheelForceFrontRight(wheel_force);
}

float PhysicsRobot::getMotorBrakeForce(float motor_speed) const {
    // We approximate the braking force of the motor with a linear relationship
    // to the current motor speed and the mass of the robot. The force is applied
    // opposite the motor's current rotation / speed.
    //
    // The scaling factor has been tuned to stop the robot in a reasonable
    // amount of time via the unit tests
    return -0.5*robot_body->GetMass()*motor_speed;
}
