#include "software/simulation/physics/physics_ball.h"

#include "shared/constants.h"
#include "software/geom/algorithms/distance.h"
#include "software/physics/physics.h"
#include "software/simulation/physics/box2d_util.h"
#include "software/simulation/physics/physics_object_user_data.h"

PhysicsBall::PhysicsBall(std::shared_ptr<b2World> world, const BallState &ball_state,
                         const double mass_kg,
                         TbotsProto::SimulatorConfig simulator_config)
    : in_flight_origin(std::nullopt),
      in_flight_distance_meters(0.0),
      flight_angle_of_departure(Angle::zero()),
      initial_kick_speed(std::nullopt),
      simulator_config(simulator_config)
{
    // All the BodyDef must be defined before the body is created.
    // Changes made after aren't reflected
    b2BodyDef ball_body_def;
    ball_body_def.type = b2_dynamicBody;
    ball_body_def.position.Set(static_cast<float>(ball_state.position().x()),
                               static_cast<float>(ball_state.position().y()));
    ball_body_def.linearVelocity.Set(static_cast<float>(ball_state.velocity().x()),
                                     static_cast<float>(ball_state.velocity().y()));
    // The ball can potentially move relatively quickly, so treating it as a "bullet"
    // helps prevent tunneling and other collision problems
    // See the "Breakdown of a collision" section of:
    // https://www.iforce2d.net/b2dtut/collision-anatomy
    ball_body_def.bullet        = true;
    ball_body_def.linearDamping = 0.0;
    ball_body                   = world->CreateBody(&ball_body_def);

    b2CircleShape ball_shape;
    ball_shape.m_radius = static_cast<float>(BALL_MAX_RADIUS_METERS);

    b2FixtureDef ball_fixture_def;
    ball_fixture_def.shape = &ball_shape;
    // Calculate the density the fixture / ball must have in order for it to have the
    // desired mass. The density is uniform across the shape.
    float ball_area =
        static_cast<float>(M_PI * ball_shape.m_radius * ball_shape.m_radius);
    ball_fixture_def.density = static_cast<float>(mass_kg / ball_area);
    ball_fixture_def.restitution =
        static_cast<float>(simulator_config.ball_restitution());
    ball_fixture_def.friction = static_cast<float>(BALL_FRICTION);
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
        for (b2Fixture *f = ball_body->GetFixtureList(); f != NULL; f = f->GetNext())
        {
            if (f->GetUserData() != NULL)
            {
                PhysicsObjectUserData *user_data =
                    static_cast<PhysicsObjectUserData *>(f->GetUserData());
                delete user_data;
                f->SetUserData(NULL);
            }
        }
        world->DestroyBody(ball_body);
    }
}

BallState PhysicsBall::getBallState() const
{
    return BallState(position(), velocity(), calculateDistanceFromGround());
}

Point PhysicsBall::position() const
{
    return createPoint(ball_body->GetPosition());
}

Vector PhysicsBall::velocity() const
{
    return createVector(ball_body->GetLinearVelocity());
}

Vector PhysicsBall::momentum() const
{
    double momentum_magnitude = massKg() * velocity().length();
    return velocity().normalize(momentum_magnitude);
}

float PhysicsBall::massKg() const
{
    return ball_body->GetMass();
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

void PhysicsBall::setInFlightForDistance(double in_flight_distance,
                                         Angle angle_of_departure)
{
    in_flight_origin          = position();
    in_flight_distance_meters = in_flight_distance;
    flight_angle_of_departure = angle_of_departure;
}

double PhysicsBall::calculateDistanceFromGround() const
{
    double distance_from_ground = 0.0;
    if (isInFlight())
    {
        // We already know the range and angle of departure, so we can re-arrange
        // to solve for the theoretical initial speed of the ball. This then lets us
        // calculate the parabolic trajectory.
        // https://courses.lumenlearning.com/boundless-physics/chapter/projectile-motion/
        double x = distance(position(), in_flight_origin.value());
        double initial_speed_numerator =
            in_flight_distance_meters *
            ACCELERATION_DUE_TO_GRAVITY_METERS_PER_SECOND_SQUARED;
        double initial_speed_denominator = (flight_angle_of_departure * 2).sin();
        double initial_speed =
            std::sqrt(initial_speed_numerator / initial_speed_denominator);
        double y_numerator =
            ACCELERATION_DUE_TO_GRAVITY_METERS_PER_SECOND_SQUARED * x * x;
        double y_denominator = 2 * initial_speed * initial_speed *
                               std::pow(flight_angle_of_departure.cos(), 2);
        double y = std::tan(flight_angle_of_departure.toRadians()) * x -
                   (y_numerator / y_denominator);

        if (isTouchingOtherObject())
        {
            // If the ball is still in flight and touching another object,
            // it must be on top of another robot/object and so cannot be
            // lower than that
            distance_from_ground = std::max(y, ROBOT_MAX_HEIGHT_METERS);
        }
        else
        {
            distance_from_ground = std::max(y, 0.0);
        }
    }

    return distance_from_ground;
}

void PhysicsBall::updateIsInFlight() const
{
    if (in_flight_origin.has_value())
    {
        double current_in_flight_distance_meters =
            (position() - in_flight_origin.value()).length();
        // Once the ball is in flight, it can only stop being in flight once it has
        // travelled at least the current in_flight_distance and is simultaneously not
        // touching another object. This prevents the ball from "landing" in another
        // object, and instead pretends the ball hit the top and rolled off.
        //
        // We assume the ball does not collide while it is in flight, which gives us the
        // "guarantee" the ball will travel far enough from the in_flight_origin in order
        // to "land"
        if (current_in_flight_distance_meters >= in_flight_distance_meters &&
            !isTouchingOtherObject())
        {
            in_flight_origin = std::nullopt;
        }
    }
}

void PhysicsBall::setInitialKickSpeed(double speed)
{
    initial_kick_speed = speed;
}

void PhysicsBall::applyBallFrictionModel(const Duration &time_step)
{
    Vector velocity_delta = calculateVelocityDeltaDueToFriction(time_step);
    applyImpulse(velocity_delta * massKg());
}

Vector PhysicsBall::calculateVelocityDeltaDueToFriction(const Duration &time_step)
{
    // Friction model adapted from section 5 of
    // https://ssl.robocup.org/wp-content/uploads/2020/03/2020_ETDP_ZJUNlict.pdf
    //
    // To summarize, because the ball is a uniform density sphere, v1 = 5 / 7 * v0,
    // where v1 is the initial rolling speed and v0 is the initial sliding speed
    static constexpr double SLIDING_ROLLING_TRANSITION_FACTOR = 5.0 / 7.0;
    const double rolling_friction_acceleration =
        simulator_config.rolling_friction_acceleration();

    auto current_ball_state = getBallState();

    if (initial_kick_speed)
    {
        double sliding_to_rolling_speed_threshold =
            *initial_kick_speed * SLIDING_ROLLING_TRANSITION_FACTOR;
        Vector future_velocity = calculateFrictionBallModelFutureVelocity(
            current_ball_state.velocity(), sliding_to_rolling_speed_threshold, time_step);
        if (future_velocity.length() < sliding_to_rolling_speed_threshold)
        {
            // initial kick speed is no longer relevant once ball is rolling
            initial_kick_speed = std::nullopt;
        }
        return future_velocity - current_ball_state.velocity();
    }
    else
    {
        return current_ball_state.velocity().normalize(-rolling_friction_acceleration *
                                                       time_step.toSeconds());
    }
}

Vector PhysicsBall::calculateFrictionBallModelFutureVelocity(
    const Vector &initial_ball_velocity, double sliding_to_rolling_speed_threshold,
    const Duration &duration_in_future) const
{
    const double seconds_in_future = duration_in_future.toSeconds();
    const double initial_speed     = initial_ball_velocity.length();
    const double sliding_friction_acceleration =
        simulator_config.sliding_friction_acceleration();
    const double rolling_friction_acceleration =
        simulator_config.rolling_friction_acceleration();

    // Figure out how long the ball will roll/slide, if at all
    // if sliding_friction_acceleration is 0 then max_sliding_duration_secs is inf,
    // which is handled by std::min
    const double max_sliding_duration_secs =
        (initial_speed - sliding_to_rolling_speed_threshold) /
        sliding_friction_acceleration;
    const double sliding_duration_secs =
        std::max(0.0, std::min(seconds_in_future, max_sliding_duration_secs));
    // if rolling_friction_acceleration is 0 then max_rolling_duration_secs is inf,
    // which is handled by std::min
    const double max_rolling_duration_secs =
        std::min(initial_speed, sliding_to_rolling_speed_threshold) /
        rolling_friction_acceleration;
    const double rolling_duration_secs = std::max(
        0.0,
        std::min(seconds_in_future - sliding_duration_secs, max_rolling_duration_secs));

    // Figure out where the ball is after sliding and rolling
    const Vector sliding_acceleration_vector =
        initial_ball_velocity.normalize(-sliding_friction_acceleration);
    const Vector velocity_after_sliding =
        calculateFutureVelocity(initial_ball_velocity, sliding_acceleration_vector,
                                Duration::fromSeconds(sliding_duration_secs));

    const Vector rolling_acceleration_vector =
        initial_ball_velocity.normalize(-rolling_friction_acceleration);
    const Vector velocity_after_rolling =
        calculateFutureVelocity(velocity_after_sliding, rolling_acceleration_vector,
                                Duration::fromSeconds(rolling_duration_secs));

    return velocity_after_rolling;
}

bool PhysicsBall::isInFlight() const
{
    updateIsInFlight();
    return in_flight_origin.has_value();
}
