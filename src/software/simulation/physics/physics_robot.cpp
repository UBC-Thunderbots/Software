#include "software/simulation/physics/physics_robot.h"

#include <algorithm>

#include "shared/constants.h"
#include "software/simulation/physics/box2d_util.h"
#include "software/simulation/physics/physics_object_user_data.h"
#include "software/simulation/physics/physics_robot_model.h"
extern "C"
{
#include "firmware/shared/physics.h"
}

// We are only allowed to cover a fraction of the ball according to the rules, so we use
// this to calculate the depth. We assume the ball can be dribbled as long as it is
// anywhere within this small area.
const double PhysicsRobot::dribbler_depth =
    BALL_MAX_RADIUS_METERS * 2 * MAX_FRACTION_OF_BALL_COVERED_BY_ROBOT;
// We can use a very small value for the chicker thickness since the
// chicker just needs to be large enough to collide with the ball and detect collisions
// without letting the ball tunnel and collide with the robot body
const double PhysicsRobot::chicker_thickness = 0.005;
const double PhysicsRobot::total_chicker_depth =
    PhysicsRobot::dribbler_depth + PhysicsRobot::chicker_thickness;

PhysicsRobot::PhysicsRobot(RobotId id, std::shared_ptr<b2World> world,
                           const RobotState& robot_state, const double mass_kg)
    : robot_id(id)
{
    b2BodyDef robot_body_def;
    robot_body_def.type = b2_dynamicBody;
    robot_body_def.position.Set(static_cast<float>(robot_state.position().x()),
                                static_cast<float>(robot_state.position().y()));
    robot_body_def.linearVelocity.Set(static_cast<float>(robot_state.velocity().x()),
                                      static_cast<float>(robot_state.velocity().y()));
    robot_body_def.angle = static_cast<float>(robot_state.orientation().toRadians());
    robot_body_def.angularVelocity =
        static_cast<float>(robot_state.angularVelocity().toRadians());
    robot_body_def.linearDamping  = static_cast<float>(robot_linear_damping);
    robot_body_def.angularDamping = static_cast<float>(robot_angular_damping);

    robot_body = world->CreateBody(&robot_body_def);

    setupRobotBodyFixtures(robot_state, PhysicsRobot::total_chicker_depth, mass_kg);
    setupDribblerFixture(robot_state, PhysicsRobot::dribbler_depth);
    setupChickerFixture(robot_state, PhysicsRobot::total_chicker_depth,
                        PhysicsRobot::chicker_thickness);

    // For some reason adding fixtures with mass slightly changes the linear velocity
    // of the body, so we make sure to reset it to the desired value at the end
    robot_body->SetLinearVelocity(createVec2(robot_state.velocity()));
}

PhysicsRobot::~PhysicsRobot()
{
    // Examples for removing bodies safely from
    // https://www.iforce2d.net/b2dtut/removing-bodies
    b2World* robot_body_world = robot_body->GetWorld();
    if (bodyExistsInWorld(robot_body, robot_body_world))
    {
        for (b2Fixture* f = robot_body->GetFixtureList(); f != NULL; f = f->GetNext())
        {
            if (f->GetUserData() != NULL)
            {
                PhysicsObjectUserData* user_data =
                    static_cast<PhysicsObjectUserData*>(f->GetUserData());
                delete user_data;
                f->SetUserData(NULL);
            }
        }
        robot_body_world->DestroyBody(robot_body);
    }
}

void PhysicsRobot::setupRobotBodyFixtures(const RobotState&, double total_chicker_depth,
                                          const double mass_kg)
{
    b2PolygonShape* main_body_shape =
        PhysicsRobotModel::getMainRobotBodyShape(total_chicker_depth);
    b2PolygonShape* front_left_body_shape =
        PhysicsRobotModel::getRobotBodyShapeFrontLeft(total_chicker_depth);
    b2PolygonShape* front_right_body_shape =
        PhysicsRobotModel::getRobotBodyShapeFrontRight(total_chicker_depth);

    auto body_shapes = {main_body_shape, front_left_body_shape, front_right_body_shape};
    double total_shape_area = 0.0;
    for (const auto* shape : body_shapes)
    {
        total_shape_area += polygonArea(*shape);
    }
    const float density = static_cast<float>(mass_kg / total_shape_area);

    for (const auto shape : body_shapes)
    {
        setupRobotBodyFixture(shape, density);
    }
    delete main_body_shape;
    delete front_left_body_shape;
    delete front_right_body_shape;
}

void PhysicsRobot::setupRobotBodyFixture(const b2PolygonShape* shape, const float density)
{
    b2FixtureDef robot_body_fixture_def;
    robot_body_fixture_def.restitution = static_cast<float>(robot_body_restitution);
    robot_body_fixture_def.friction    = static_cast<float>(robot_body_friction);
    robot_body_fixture_def.userData =
        new PhysicsObjectUserData({PhysicsObjectType::ROBOT_BODY, this});
    robot_body_fixture_def.density = density;
    robot_body_fixture_def.shape   = shape;
    robot_body->CreateFixture(&robot_body_fixture_def);
}

void PhysicsRobot::setupDribblerFixture(const RobotState&, double dribbler_depth)
{
    b2FixtureDef robot_dribbler_fixture_def;
    robot_dribbler_fixture_def.density = static_cast<float>(robot_dribbler_density);
    // We explicitly choose to make the dribbler NOT a sensor, because sensor fixtures do
    // not trigger PreSolve contact callbacks, which we rely on to apply dribbling force
    // at every physics step
    robot_dribbler_fixture_def.isSensor = false;
    robot_dribbler_fixture_def.restitution =
        static_cast<float>(robot_dribbler_restitution);
    robot_dribbler_fixture_def.friction = static_cast<float>(robot_dribbler_friction);
    robot_dribbler_fixture_def.userData =
        new PhysicsObjectUserData({PhysicsObjectType::ROBOT_DRIBBLER, this});

    // Box2D requires that polygon vertices are specified in counter-clockwise order.
    // The fixture shape is added relative to the body in its local coordinate frame,
    // so we do not need to rotate the points to match the orientation of the robot.
    const unsigned int num_vertices              = 4;
    b2Vec2 dribbler_shape_vertices[num_vertices] = {
        createVec2(Point(DIST_TO_FRONT_OF_ROBOT_METERS, DRIBBLER_WIDTH_METERS / 2.0)),
        createVec2(Point(DIST_TO_FRONT_OF_ROBOT_METERS - dribbler_depth,
                         DRIBBLER_WIDTH_METERS / 2.0)),
        createVec2(Point(DIST_TO_FRONT_OF_ROBOT_METERS - dribbler_depth,
                         -DRIBBLER_WIDTH_METERS / 2.0)),
        createVec2(Point(DIST_TO_FRONT_OF_ROBOT_METERS, -DRIBBLER_WIDTH_METERS / 2.0))};
    b2PolygonShape* dribbler_shape = new b2PolygonShape();
    dribbler_shape->Set(dribbler_shape_vertices, num_vertices);
    robot_dribbler_fixture_def.shape = dribbler_shape;
    robot_body->CreateFixture(&robot_dribbler_fixture_def);
    delete dribbler_shape;
}

void PhysicsRobot::setupChickerFixture(const RobotState&, double total_chicker_depth,
                                       double chicker_thickness)
{
    b2FixtureDef robot_chicker_fixture_def;
    robot_chicker_fixture_def.restitution = static_cast<float>(robot_chicker_restitution);
    robot_chicker_fixture_def.friction    = static_cast<float>(robot_chicker_friction);
    robot_chicker_fixture_def.density     = static_cast<float>(robot_chicker_density);
    robot_chicker_fixture_def.userData =
        new PhysicsObjectUserData({PhysicsObjectType::ROBOT_CHICKER, this});

    // Box2D requires that polygon vertices are specified in counter-clockwise order.
    // The fixture shape is added relative to the body in its local coordinate frame,
    // so we do not need to rotate the points to match the orientation of the robot.
    const unsigned int num_vertices             = 4;
    b2Vec2 chicker_shape_vertices[num_vertices] = {
        createVec2(
            Point(DIST_TO_FRONT_OF_ROBOT_METERS - total_chicker_depth + chicker_thickness,
                  DRIBBLER_WIDTH_METERS / 2.0)),
        createVec2(Point(DIST_TO_FRONT_OF_ROBOT_METERS - total_chicker_depth,
                         DRIBBLER_WIDTH_METERS / 2.0)),
        createVec2(Point(DIST_TO_FRONT_OF_ROBOT_METERS - total_chicker_depth,
                         -DRIBBLER_WIDTH_METERS / 2.0)),
        createVec2(
            Point(DIST_TO_FRONT_OF_ROBOT_METERS - total_chicker_depth + chicker_thickness,
                  -DRIBBLER_WIDTH_METERS / 2.0))};

    b2PolygonShape* chicker_shape = new b2PolygonShape();
    chicker_shape->Set(chicker_shape_vertices, num_vertices);
    robot_chicker_fixture_def.shape = chicker_shape;
    robot_body->CreateFixture(&robot_chicker_fixture_def);
    delete chicker_shape;
}

RobotId PhysicsRobot::getRobotId() const
{
    return robot_id;
}

void PhysicsRobot::registerDribblerBallContactCallback(
    std::function<void(PhysicsRobot*, PhysicsBall*)> callback)
{
    dribbler_ball_contact_callbacks.emplace_back(callback);
}

void PhysicsRobot::registerDribblerBallStartContactCallback(
    std::function<void(PhysicsRobot*, PhysicsBall*)> callback)
{
    dribbler_ball_start_contact_callbacks.emplace_back(callback);
}

void PhysicsRobot::registerDribblerBallEndContactCallback(
    std::function<void(PhysicsRobot*, PhysicsBall*)> callback)
{
    dribbler_ball_end_contact_callbacks.emplace_back(callback);
}

void PhysicsRobot::registerChickerBallStartContactCallback(
    std::function<void(PhysicsRobot*, PhysicsBall*)> callback)
{
    chicker_ball_contact_callbacks.emplace_back(callback);
}

std::vector<std::function<void(PhysicsRobot*, PhysicsBall*)>>
PhysicsRobot::getDribblerBallContactCallbacks() const
{
    return dribbler_ball_contact_callbacks;
}

std::vector<std::function<void(PhysicsRobot*, PhysicsBall*)>>
PhysicsRobot::getDribblerBallStartContactCallbacks() const
{
    return dribbler_ball_start_contact_callbacks;
}

std::vector<std::function<void(PhysicsRobot*, PhysicsBall*)>>
PhysicsRobot::getDribblerBallEndContactCallbacks() const
{
    return dribbler_ball_end_contact_callbacks;
}

std::vector<std::function<void(PhysicsRobot*, PhysicsBall*)>>
PhysicsRobot::getChickerBallStartContactCallbacks() const
{
    return chicker_ball_contact_callbacks;
}

RobotState PhysicsRobot::getRobotState() const
{
    return RobotState(position(), velocity(), orientation(), angularVelocity());
}

Point PhysicsRobot::position() const
{
    return Point(robot_body->GetPosition().x, robot_body->GetPosition().y);
}

Vector PhysicsRobot::velocity() const
{
    return Vector(robot_body->GetLinearVelocity().x, robot_body->GetLinearVelocity().y);
}

Angle PhysicsRobot::orientation() const
{
    return Angle::fromRadians(robot_body->GetAngle());
}

AngularVelocity PhysicsRobot::angularVelocity() const
{
    return AngularVelocity::fromRadians(robot_body->GetAngularVelocity());
}

void PhysicsRobot::applyWheelForceFrontLeft(double force_in_newtons)
{
    Angle angle_to_wheel = Angle::fromDegrees(ANGLE_TO_ROBOT_FRONT_WHEELS_DEG);
    applyWheelForceAtAngle(angle_to_wheel, force_in_newtons);
}

void PhysicsRobot::applyWheelForceBackLeft(double force_in_newtons)
{
    Angle angle_to_wheel = Angle::fromDegrees(ANGLE_TO_ROBOT_BACK_WHEELS_DEG);
    applyWheelForceAtAngle(angle_to_wheel, force_in_newtons);
}

void PhysicsRobot::applyWheelForceBackRight(double force_in_newtons)
{
    Angle angle_to_wheel = Angle::fromDegrees(-ANGLE_TO_ROBOT_BACK_WHEELS_DEG);
    applyWheelForceAtAngle(angle_to_wheel, force_in_newtons);
}

void PhysicsRobot::applyWheelForceFrontRight(double force_in_newtons)
{
    Angle angle_to_wheel = Angle::fromDegrees(-ANGLE_TO_ROBOT_FRONT_WHEELS_DEG);
    applyWheelForceAtAngle(angle_to_wheel, force_in_newtons);
}

void PhysicsRobot::applyWheelForceAtAngle(Angle angle_to_wheel, double force_in_newtons)
{
    // The center of the robot is always at (0, 0) in its own coordinate frame
    Point local_robot_position = Point(0, 0);
    Point local_force_point =
        local_robot_position +
        Vector::createFromAngle(angle_to_wheel).normalize(ROBOT_MAX_RADIUS_METERS);
    Vector local_force_vector =
        (local_force_point - local_robot_position).perpendicular();
    local_force_vector  = local_force_vector.normalize(force_in_newtons);
    b2Vec2 force_point  = robot_body->GetWorldPoint(createVec2(local_force_point));
    b2Vec2 force_vector = robot_body->GetWorldVector(createVec2(local_force_vector));
    robot_body->ApplyForce(force_vector, force_point, true);
}

std::array<float, 4> PhysicsRobot::getMotorSpeeds() const
{
    float robot_local_speed[3]{
        robot_body->GetLocalVector(robot_body->GetLinearVelocity()).x,
        robot_body->GetLocalVector(robot_body->GetLinearVelocity()).y,
        robot_body->GetAngularVelocity()};
    float wheel_speeds[4]{0.0, 0.0, 0.0, 0.0};
    speed3_to_speed4(robot_local_speed, wheel_speeds);
    std::array<float, 4> motor_speeds = {0.0, 0.0, 0.0, 0.0};
    for (unsigned int i = 0; i < 4; i++)
    {
        motor_speeds[i] = wheel_speeds[i] / GEAR_RATIO;
    }
    return motor_speeds;
}

float PhysicsRobot::getMotorSpeedFrontLeft()
{
    return getMotorSpeeds()[0];
}

float PhysicsRobot::getMotorSpeedBackLeft()
{
    return getMotorSpeeds()[1];
}

float PhysicsRobot::getMotorSpeedBackRight()
{
    return getMotorSpeeds()[2];
}

float PhysicsRobot::getMotorSpeedFrontRight()
{
    return getMotorSpeeds()[3];
}

void PhysicsRobot::brakeMotorFrontLeft()
{
    float motor_speed = getMotorSpeedFrontLeft();
    float wheel_force = getMotorBrakeForce(motor_speed);
    applyWheelForceFrontLeft(wheel_force);
}

void PhysicsRobot::brakeMotorBackLeft()
{
    float motor_speed = getMotorSpeedBackLeft();
    float wheel_force = getMotorBrakeForce(motor_speed);
    applyWheelForceBackLeft(wheel_force);
}

void PhysicsRobot::brakeMotorBackRight()
{
    float motor_speed = getMotorSpeedBackRight();
    float wheel_force = getMotorBrakeForce(motor_speed);
    applyWheelForceBackRight(wheel_force);
}

void PhysicsRobot::brakeMotorFrontRight()
{
    float motor_speed = getMotorSpeedFrontRight();
    float wheel_force = getMotorBrakeForce(motor_speed);
    applyWheelForceFrontRight(wheel_force);
}

float PhysicsRobot::getMotorBrakeForce(float motor_speed) const
{
    // We approximate the braking force of the motor with a linear relationship
    // to the current motor speed and the mass of the robot. The force is applied
    // opposite the motor's current rotation / speed.
    //
    // The scaling factor has been tuned to stop the robot in a reasonable
    // amount of time via the unit tests
    return -0.5f * robot_body->GetMass() * motor_speed;
}

void PhysicsRobot::setPosition(const Point& position)
{
    auto func = [=]() {
        b2World* world = robot_body->GetWorld();
        if (bodyExistsInWorld(robot_body, world))
        {
            robot_body->SetLinearVelocity(createVec2(Vector(0, 0)));
            robot_body->SetAngularVelocity(0.0);
            robot_body->SetTransform(createVec2(position), robot_body->GetAngle());
        }
    };

    // We can't set the robot body's position immediately because we may be in the middle
    // of a Box2D world update, so we defer calling this function until after a physics
    // step
    post_physics_step_functions.emplace(func);
}

void PhysicsRobot::runPostPhysicsStep()
{
    while (!post_physics_step_functions.empty())
    {
        auto post_physics_step_function = post_physics_step_functions.front();
        post_physics_step_function();
        post_physics_step_functions.pop();
    }
}
