#include "software/backend/simulation/physics/physics_ball.h"

#include <cmath>

#include "shared/constants.h"
#include "software/backend/simulation/physics/box2d_util.h"
#include "software/backend/simulation/physics/physics_object_user_data.h"

PhysicsBall::PhysicsBall(std::shared_ptr<b2World> world, const Ball &ball, double mass_kg) : simulator_ball(nullptr)
{
    // All the BodyDef must be defined before the body is created.
    // Changes made after aren't reflected
    ball_body_def.type = b2_dynamicBody;
    ball_body_def.position.Set(ball.position().x(), ball.position().y());
    ball_body_def.linearVelocity.Set(ball.velocity().x(), ball.velocity().y());
    // The ball can potentially move relatively quickly, so treating it as a "bullet"
    // helps prevent tunneling and other collision problems
    // See the "Breakdown of a collision" section of:
    // https://www.iforce2d.net/b2dtut/collision-anatomy
    ball_body_def.bullet = true;
    ball_body            = world->CreateBody(&ball_body_def);

    ball_shape.m_radius = BALL_MAX_RADIUS_METERS;

    ball_fixture_def.shape = &ball_shape;

    // Calculate the density the fixture / ball must have in order for it to have the
    // desired mass. The density is uniform across the shape.
    float ball_area          = M_PI * ball_shape.m_radius * ball_shape.m_radius;
    ball_fixture_def.density = mass_kg / ball_area;
    // These restitution and friction values are somewhat arbitrary. Because this is an
    // "ideal" simulation, we can approximate the ball as having perfectly elastic
    // collisions and no friction. Because we also do not generally depend on specific
    // behaviour when the ball collides with something, getting these values to perfectly
    // match reality isn't too important.
    ball_fixture_def.restitution = 1.0;
    ball_fixture_def.friction    = 0.0;
    ball_fixture_def.userData = new PhysicsObjectUserData({PhysicsObjectType::BALL, this});

    ball_body->CreateFixture(&ball_fixture_def);
}

PhysicsBall::~PhysicsBall()
{
    // Examples for removing bodies safely from
    // https://www.iforce2d.net/b2dtut/removing-bodies
    b2World *world = ball_body->GetWorld();
    if (bodyExistsInWorld(ball_body, world))
    {
        world->DestroyBody(ball_body);
    }
}

Ball PhysicsBall::getBallWithTimestamp(const Timestamp &timestamp) const
{
    auto position = Point(ball_body->GetPosition().x, ball_body->GetPosition().y);
    auto velocity =
        Vector(ball_body->GetLinearVelocity().x, ball_body->GetLinearVelocity().y);
    return Ball(position, velocity, timestamp);
}

double PhysicsBall::getMassKg() const {
    return static_cast<double>(ball_body->GetMass());
}

void PhysicsBall::applyForce(const Vector& force) {
    b2Vec2 force_vector = createVec2(force);
    ball_body->ApplyForce(force_vector, ball_body->GetWorldCenter(), true);
}

void PhysicsBall::applyImpulse(const Vector& impulse) {
    b2Vec2 impulse_vector = createVec2(impulse);
    ball_body->ApplyLinearImpulseToCenter(impulse_vector, true);
}

void PhysicsBall::setSimulatorBall(SimulatorBall *simulator_ball) {
    this->simulator_ball = simulator_ball;
}

SimulatorBall* PhysicsBall::getSimulatorBall() const {
    return simulator_ball;
}
