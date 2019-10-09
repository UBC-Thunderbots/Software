#include "software/backend/simulation/physics_ball.h"

#include "shared/constants.h"
#include "software/backend/simulation/box2d_util.h"

PhysicsBall::PhysicsBall(std::shared_ptr<b2World> world, const Ball &ball)
{
    // All the BodyDef must be defined before the body is created.
    // Changes made after aren't reflected
    ball_body_def.type = b2_dynamicBody;
    ball_body_def.position.Set(ball.position().x(), ball.position().y());
    ball_body_def.linearVelocity.Set(ball.velocity().x(), ball.velocity().y());
    // The ball can potentially move relatively quickly, so treating it as a "bullet"
    // helps prevent tunneling and other collision problems
    ball_body_def.bullet = true;
    ball_body            = world->CreateBody(&ball_body_def);

    ball_shape.m_radius = BALL_MAX_RADIUS_METERS;

    ball_fixture_def.shape       = &ball_shape;
    ball_fixture_def.density     = 1.0;
    ball_fixture_def.restitution = 1.0;
    ball_fixture_def.friction    = 0.0;

    ball_body->CreateFixture(&ball_fixture_def);
}

Ball PhysicsBall::getBallWithTimestamp(const Timestamp &timestamp) const
{
    auto position = Point(ball_body->GetPosition().x, ball_body->GetPosition().y);
    auto velocity =
        Vector(ball_body->GetLinearVelocity().x, ball_body->GetLinearVelocity().y);
    return Ball(position, velocity, timestamp);
}

void PhysicsBall::removeFromWorld(std::shared_ptr<b2World> world)
{
    if (bodyExistsInWorld(ball_body, world))
    {
        world->DestroyBody(ball_body);
    }
}
