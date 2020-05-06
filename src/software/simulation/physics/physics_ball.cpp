#include "software/simulation/physics/physics_ball.h"

#include <cmath>

#include "shared/constants.h"
#include "software/simulation/physics/box2d_util.h"
#include "software/simulation/physics/physics_object_user_data.h"

PhysicsBall::PhysicsBall(std::shared_ptr<b2World> world, const Ball &ball, double mass_kg,
                         const double gravity)
    : gravity(gravity), chip_origin(std::nullopt), chip_distance_meters(0.0)
{
    // All the BodyDef must be defined before the body is created.
    // Changes made after aren't reflected
    b2BodyDef ball_body_def;
    ball_body_def.type = b2_dynamicBody;
    ball_body_def.position.Set(ball.position().x(), ball.position().y());
    ball_body_def.linearVelocity.Set(ball.velocity().x(), ball.velocity().y());
    // The ball can potentially move relatively quickly, so treating it as a "bullet"
    // helps prevent tunneling and other collision problems
    // See the "Breakdown of a collision" section of:
    // https://www.iforce2d.net/b2dtut/collision-anatomy
    ball_body_def.bullet = true;
    ball_body            = world->CreateBody(&ball_body_def);

    b2CircleShape ball_shape;
    ball_shape.m_radius = BALL_MAX_RADIUS_METERS;

    b2FixtureDef ball_fixture_def;
    ball_fixture_def.shape = &ball_shape;
    // Calculate the density the fixture / ball must have in order for it to have the
    // desired mass. The density is uniform across the shape.
    float ball_area              = M_PI * ball_shape.m_radius * ball_shape.m_radius;
    ball_fixture_def.density     = mass_kg / ball_area;
    ball_fixture_def.restitution = ball_restitution;
    ball_fixture_def.friction    = ball_friction;
    ball_fixture_def.userData =
        new PhysicsObjectUserData({PhysicsObjectType::BALL, this});

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
    return Ball(position(), velocity(), timestamp);
}

Point PhysicsBall::position() const
{
    return createPoint(ball_body->GetPosition());
}

Vector PhysicsBall::velocity() const
{
    return createVector(ball_body->GetLinearVelocity());
}

void PhysicsBall::kick(Vector kick_vector)
{
    // Figure out how much impulse to apply to change the speed of the ball by the
    // magnitude of the kick_vector
    double change_in_momentum = ball_body->GetMass() * kick_vector.length();
    kick_vector               = kick_vector.normalize(change_in_momentum);
    applyImpulse(kick_vector);
}

void PhysicsBall::chip(const Vector &chip_vector)
{
    // Assume the ball is chipped at a 45 degree angle
    // TODO: Use a robot-specific constant
    // https://github.com/UBC-Thunderbots/Software/issues/1179
    Angle chip_angle = Angle::fromDegrees(45);
    // Use the formula for the Range of a parabolic projectile
    // Rearrange to solve for the initial velocity
    // See https://courses.lumenlearning.com/boundless-physics/chapter/projectile-motion/
    double range            = chip_vector.length();
    double numerator        = range * gravity;
    double denominator      = 2 * (chip_angle * 2).sin();
    double initial_velocity = std::sqrt(numerator / denominator);
    double ground_velocity  = initial_velocity * chip_angle.cos();
    kick(chip_vector.normalize(ground_velocity));
    chip_origin          = getBallWithTimestamp(Timestamp::fromSeconds(0)).position();
    chip_distance_meters = chip_vector.length();
}

void PhysicsBall::applyForce(const Vector &force)
{
    b2Vec2 force_vector = createVec2(force);
    ball_body->ApplyForce(force_vector, ball_body->GetWorldCenter(), true);
}

void PhysicsBall::applyImpulse(const Vector &impulse)
{
    b2Vec2 impulse_vector = createVec2(impulse);
    ball_body->ApplyLinearImpulseToCenter(impulse_vector, true);
}

bool PhysicsBall::isTouchingOtherObject() const
{
    b2ContactEdge *contact_edge = ball_body->GetContactList();
    while (contact_edge != nullptr)
    {
        if (contact_edge->contact->IsTouching())
        {
            return true;
        }
        contact_edge = contact_edge->next;
    }
    return false;
}

bool PhysicsBall::isInFlight()
{
    bool chip_in_progress = chip_origin.has_value();
    if (chip_in_progress)
    {
        double current_chip_distance_meters =
            (getBallWithTimestamp(Timestamp::fromSeconds(0)).position() -
             chip_origin.value())
                .length();
        // Once the ball is in flight, is can only stop being in flight once it has
        // travelled at least the current chip_distance and is simultaneously not touching
        // another object. This prevents the ball from "landing" in another object, and
        // instead pretends the ball hit the top and rolled off.
        //
        // We assume the ball does not collide while it is in flight, which gives us the
        // "guarantee" the ball will travel far enough from the chip_origin in order to
        // "land"
        if (current_chip_distance_meters >= chip_distance_meters &&
            !isTouchingOtherObject())
        {
            chip_origin = std::nullopt;
            return false;
        }
        else
        {
            return true;
        }
    }

    return false;
}
