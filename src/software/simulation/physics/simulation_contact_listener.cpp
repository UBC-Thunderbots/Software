#include "software/simulation/physics/simulation_contact_listener.h"

void SimulationContactListener::BeginContact(b2Contact *contact)
{
    if (contact->IsTouching())
    {
        auto fixture_a = contact->GetFixtureA();
        auto fixture_b = contact->GetFixtureB();

        PhysicsObjectUserData *user_data_a =
            static_cast<PhysicsObjectUserData *>(fixture_a->GetUserData());
        PhysicsObjectUserData *user_data_b =
            static_cast<PhysicsObjectUserData *>(fixture_b->GetUserData());

        if (auto ball = isBallContact(user_data_a, user_data_b))
        {
            // Disable collisions with the ball if it is in flight. This is how we
            // simulate the ball being chipped any flying over other objects
            // in a 2D simulation
            if (ball->isInFlight())
            {
                contact->SetEnabled(false);
                return;
            }
        }

        if (auto ball_dribbler_pair = isDribblerBallContact(user_data_a, user_data_b))
        {
            PhysicsBall *ball   = ball_dribbler_pair->first;
            PhysicsRobot *robot = ball_dribbler_pair->second;
            for (auto contact_callback : robot->getDribblerBallStartContactCallbacks())
            {
                contact_callback(robot, ball);
            }
        }
        if (auto ball_field_pair = isBallFieldWallContact(user_data_a, user_data_b))
        {
            // If the ball hits a wall it's out of bounds, so we stop it to make it easier
            // to retrieve for ball placement (otherwise it will bounce around the field)
            PhysicsBall *ball = ball_field_pair->first;
            // Apply an impulse to nearly stop the ball. It's preferable if it bounces
            // away from a wall very slightly for realism, and it should make it easier to
            // retrieve for ball placement
            ball->applyImpulse(
                -ball->momentum().normalize(ball->momentum().length() * 0.95));
        }
    }
}

void SimulationContactListener::PreSolve(b2Contact *contact,
                                         const b2Manifold *oldManifold)
{
    if (contact->IsTouching())
    {
        auto fixture_a = contact->GetFixtureA();
        auto fixture_b = contact->GetFixtureB();

        PhysicsObjectUserData *user_data_a =
            static_cast<PhysicsObjectUserData *>(fixture_a->GetUserData());
        PhysicsObjectUserData *user_data_b =
            static_cast<PhysicsObjectUserData *>(fixture_b->GetUserData());

        if (auto ball = isBallContact(user_data_a, user_data_b))
        {
            // Disable collisions with the ball if it is in flight. This is how we
            // simulate the ball being chipped any flying over other objects
            // in a 2D simulation
            if (ball->isInFlight())
            {
                contact->SetEnabled(false);
                return;
            }
        }

        if (auto ball_dribbler_pair = isDribblerBallContact(user_data_a, user_data_b))
        {
            // We always disable contacts between the ball and the dribbler so that the
            // ball can pass through the dribbler area. This is needed so that the ball
            // can exist inside the dribbling "zone", as well as pass through the dribbler
            // to make contact with the dribbler damper
            contact->SetEnabled(false);
            PhysicsBall *ball   = ball_dribbler_pair->first;
            PhysicsRobot *robot = ball_dribbler_pair->second;
            for (auto contact_callback : robot->getDribblerBallContactCallbacks())
            {
                contact_callback(robot, ball);
            }
        }
        if (auto ball_dribbler_damper_pair =
                isDribblerDamperBallContact(user_data_a, user_data_b))
        {
            // Ensure that the ball is perfectly damped if it collides with the dribbler
            // damper. This helps the robot keep the ball while dribbling.
            contact->SetRestitution(0.0);
        }
    }
}

void SimulationContactListener::EndContact(b2Contact *contact)
{
    auto fixture_a = contact->GetFixtureA();
    auto fixture_b = contact->GetFixtureB();

    PhysicsObjectUserData *user_data_a =
        static_cast<PhysicsObjectUserData *>(fixture_a->GetUserData());
    PhysicsObjectUserData *user_data_b =
        static_cast<PhysicsObjectUserData *>(fixture_b->GetUserData());

    if (auto ball = isBallContact(user_data_a, user_data_b))
    {
        // Disable collisions with the ball if it is in flight. This is how we
        // simulate the ball being chipped any flying over other objects
        // in a 2D simulation
        if (ball->isInFlight())
        {
            contact->SetEnabled(false);
            return;
        }
    }

    if (auto ball_dribbler_pair = isDribblerBallContact(user_data_a, user_data_b))
    {
        PhysicsBall *ball   = ball_dribbler_pair->first;
        PhysicsRobot *robot = ball_dribbler_pair->second;
        for (auto contact_callback : robot->getDribblerBallEndContactCallbacks())
        {
            contact_callback(robot, ball);
        }
    }
}

std::optional<std::pair<PhysicsBall *, PhysicsRobot *>>
SimulationContactListener::isDribblerDamperBallContact(PhysicsObjectUserData *user_data_a,
                                                       PhysicsObjectUserData *user_data_b)
{
    if (!user_data_a || !user_data_b)
    {
        return std::nullopt;
    }

    // NOTE: Box2D intelligently reports only one contact when 2 objects collide
    // (ie. Does not report a contact for ObjectA touching ObjectB, and another
    // contact for ObjectB touching ObjectA), which is why we do not need to worry
    // about only detecting one contact per pair of colliding objects.
    PhysicsBall *ball = nullptr;
    if (user_data_a->type == PhysicsObjectType::BALL)
    {
        ball = static_cast<PhysicsBall *>(user_data_a->physics_object);
    }
    if (user_data_b->type == PhysicsObjectType::BALL)
    {
        ball = static_cast<PhysicsBall *>(user_data_b->physics_object);
    }

    PhysicsRobot *robot = nullptr;
    if (user_data_a->type == PhysicsObjectType::ROBOT_CHICKER)
    {
        robot = static_cast<PhysicsRobot *>(user_data_a->physics_object);
    }
    if (user_data_b->type == PhysicsObjectType::ROBOT_CHICKER)
    {
        robot = static_cast<PhysicsRobot *>(user_data_b->physics_object);
    }

    if (ball && robot)
    {
        return std::make_pair(ball, robot);
    }

    return std::nullopt;
}

std::optional<std::pair<PhysicsBall *, PhysicsRobot *>>
SimulationContactListener::isDribblerBallContact(PhysicsObjectUserData *user_data_a,
                                                 PhysicsObjectUserData *user_data_b)
{
    if (!user_data_a || !user_data_b)
    {
        return std::nullopt;
    }

    // NOTE: Box2D intelligently reports only one contact when 2 objects collide
    // (ie. Does not report a contact for ObjectA touching ObjectB, and another
    // contact for ObjectB touching ObjectA), which is why we do not need to worry
    // about only detecting one contact per pair of colliding objects.
    PhysicsBall *ball = nullptr;
    if (user_data_a->type == PhysicsObjectType::BALL)
    {
        ball = static_cast<PhysicsBall *>(user_data_a->physics_object);
    }
    if (user_data_b->type == PhysicsObjectType::BALL)
    {
        ball = static_cast<PhysicsBall *>(user_data_b->physics_object);
    }

    PhysicsRobot *robot = nullptr;
    if (user_data_a->type == PhysicsObjectType::ROBOT_DRIBBLER)
    {
        robot = static_cast<PhysicsRobot *>(user_data_a->physics_object);
    }
    if (user_data_b->type == PhysicsObjectType::ROBOT_DRIBBLER)
    {
        robot = static_cast<PhysicsRobot *>(user_data_b->physics_object);
    }

    if (ball && robot)
    {
        return std::make_pair(ball, robot);
    }

    return std::nullopt;
}

PhysicsBall *SimulationContactListener::isBallContact(PhysicsObjectUserData *user_data_a,
                                                      PhysicsObjectUserData *user_data_b)
{
    PhysicsBall *ball = nullptr;
    if (user_data_a && user_data_a->type == PhysicsObjectType::BALL)
    {
        ball = static_cast<PhysicsBall *>(user_data_a->physics_object);
    }
    if (user_data_b && user_data_b->type == PhysicsObjectType::BALL)
    {
        ball = static_cast<PhysicsBall *>(user_data_b->physics_object);
    }

    return ball;
}


std::optional<std::pair<PhysicsBall *, PhysicsField *>>
SimulationContactListener::isBallFieldWallContact(PhysicsObjectUserData *user_data_a,
                                                  PhysicsObjectUserData *user_data_b)
{
    if (!user_data_a || !user_data_b)
    {
        return std::nullopt;
    }

    // NOTE: Box2D intelligently reports only one contact when 2 objects collide
    // (ie. Does not report a contact for ObjectA touching ObjectB, and another
    // contact for ObjectB touching ObjectA), which is why we do not need to worry
    // about only detecting one contact per pair of colliding objects.
    PhysicsBall *ball = nullptr;
    if (user_data_a->type == PhysicsObjectType::BALL)
    {
        ball = static_cast<PhysicsBall *>(user_data_a->physics_object);
    }
    if (user_data_b->type == PhysicsObjectType::BALL)
    {
        ball = static_cast<PhysicsBall *>(user_data_b->physics_object);
    }

    PhysicsField *field = nullptr;
    if (user_data_a->type == PhysicsObjectType::FIELD_WALL)
    {
        field = static_cast<PhysicsField *>(user_data_a->physics_object);
    }
    if (user_data_b->type == PhysicsObjectType::FIELD_WALL)
    {
        field = static_cast<PhysicsField *>(user_data_b->physics_object);
    }

    if (ball && field)
    {
        return std::make_pair(ball, field);
    }

    return std::nullopt;
}
