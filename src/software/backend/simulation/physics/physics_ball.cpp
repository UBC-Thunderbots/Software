#include "software/backend/simulation/physics/physics_ball.h"

#include <cmath>

#include "shared/constants.h"
#include "software/backend/simulation/physics/box2d_util.h"
#include "software/backend/simulation/physics/physics_object_user_data.h"

PhysicsBall::PhysicsBall(std::shared_ptr<b2World> world, const Ball &ball, double mass_kg, const double gravity) : gravity(gravity), chip_origin(std::nullopt), chip_distance_m(0.0), num_current_collisions(0)
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

void PhysicsBall::registerBallContactCallback(std::function<void(PhysicsBall *)> callback) {
    ball_contact_callbacks.emplace_back(callback);
}

std::vector<std::function<void(PhysicsBall*)>> PhysicsBall::getBallContactCallbacks() const {
    return ball_contact_callbacks;
}

Ball PhysicsBall::getBallWithTimestamp(const Timestamp &timestamp) const
{
    auto position = Point(ball_body->GetPosition().x, ball_body->GetPosition().y);
    auto velocity =
        Vector(ball_body->GetLinearVelocity().x, ball_body->GetLinearVelocity().y);
    return Ball(position, velocity, timestamp);
}

void PhysicsBall::kick(Vector kick_vector) {
    // Figure out how much impulse to apply to change the speed of the ball by the magnitude
    // of the kick_vector
    double change_in_momentum = ball_body->GetMass() * kick_vector.length();
    kick_vector = kick_vector.normalize(change_in_momentum);
    applyImpulse(kick_vector);
}

void PhysicsBall::chip(const Vector &chip_vector) {
    // Assume the ball is chipped at at a 45 degree angle
    Angle chip_angle = Angle::fromDegrees(45);
    // Use the formula for the Range of a parabolic projectile
    // Rearrange to solve for the initial velocity
    double range = chip_vector.length();
    double numerator = range * gravity;
    double denominator = 2 * (chip_angle * 2).sin();
    double initial_velocity = std::sqrt(numerator / denominator);
    double ground_velocity = initial_velocity * chip_angle.cos();
    kick(chip_vector.normalize(ground_velocity));
    chip_origin = getBallWithTimestamp(Timestamp::fromSeconds(0)).position();
    chip_distance_m = chip_vector.length();
}

void PhysicsBall::applyForce(const Vector& force) {
    b2Vec2 force_vector = createVec2(force);
    ball_body->ApplyForce(force_vector, ball_body->GetWorldCenter(), true);
}

void PhysicsBall::applyImpulse(const Vector& impulse) {
    b2Vec2 impulse_vector = createVec2(impulse);
    ball_body->ApplyLinearImpulseToCenter(impulse_vector, true);
}

bool PhysicsBall::isInFlight() {
    bool chip_in_progress = chip_origin.has_value();
    if(chip_in_progress) {
        double current_chip_distance = (getBallWithTimestamp(Timestamp::fromSeconds(0)).position() - chip_origin.value()).length();
        // TODO: better comment
        // The ball is only not in flight if it has travelled far enough to hit the ground, and
        // is not overlapping any objects when it lands. Once it "lands" it waits until it is no longer
        // colliding before reporting not in flight, which simulate the ball landing on top of another
        // object and rolling off it, such as a robot
        if(current_chip_distance >= chip_distance_m && num_current_collisions == 0) {
            chip_origin = std::nullopt;
            return false;
        }
        else {
            return true;
        }
    }

    return false;
}

void PhysicsBall::incrementNumCurrentCollisions() {
    num_current_collisions++;
}

void PhysicsBall::decrementNumCurrentCollisions() {
    if(num_current_collisions > 0) {
        num_current_collisions--;
    }
}
