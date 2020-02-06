#include "software/backend/simulation/physics/simulation_contact_listener.h"

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

        if (auto ball_chicker_pair = isBallChickerContact(user_data_a, user_data_b))
        {
            PhysicsBall* ball = ball_chicker_pair->first;
            PhysicsRobot* robot = ball_chicker_pair->second;
            for(auto contact_callback : robot->getChickerBallContactCallbacks()) {
                contact_callback(robot, ball);
            }
        }
        if(auto ball = isBallContact(user_data_a, user_data_b)) {
            ball->incrementNumCurrentCollisions();
        }
    }
}

void SimulationContactListener::PreSolve(b2Contact *contact, const b2Manifold *oldManifold) {
    if (contact->IsTouching())
    {
        auto fixture_a = contact->GetFixtureA();
        auto fixture_b = contact->GetFixtureB();

        PhysicsObjectUserData *user_data_a =
                static_cast<PhysicsObjectUserData *>(fixture_a->GetUserData());
        PhysicsObjectUserData *user_data_b =
                static_cast<PhysicsObjectUserData *>(fixture_b->GetUserData());

        if(auto ball_dribbler_pair = isBallDribblerContact(user_data_a, user_data_b))
        {
//            std::cout << "dribbler contact" << std::endl;
            // TODO: comment
            // Always disable contacts between the ball and dribbler
            contact->SetEnabled(false);
            PhysicsBall* ball = ball_dribbler_pair->first;
            PhysicsRobot* robot = ball_dribbler_pair->second;
            for(auto contact_callback : robot->getDribblerBallContactCallbacks()) {
                contact_callback(robot, ball);
            }
        }
        if (auto ball_chicker_pair = isBallChickerContact(user_data_a, user_data_b))
        {
//            std::cout << "chicker contact" << std::endl;
            contact->SetRestitution(0.0);
        }
        if(auto ball_robot_pair = isBallRobotBodyContact(user_data_a, user_data_b)) {
//            std::cout << "robot body contact" << std::endl;
//            contact->SetRestitution(0.0);
        }
        if(auto ball = isBallContact(user_data_a, user_data_b)) {
            if(ball->isInFlight()) {
                contact->SetEnabled(false);
            }
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

    if(auto ball = isBallContact(user_data_a, user_data_b)) {
        ball->decrementNumCurrentCollisions();
    }
}

std::optional<std::pair<PhysicsBall*, PhysicsRobot*>> SimulationContactListener::isBallChickerContact(PhysicsObjectUserData *user_data_a,
                                                     PhysicsObjectUserData *user_data_b)
{
    if(!user_data_a || !user_data_b) {
        return std::nullopt;
    }

    // NOTE: Box2D intelligently reports only one contact when 2 objects collide
    // (ie. Does not report a contact for ObjectA touching ObjectB, and another
    // contact for ObjectB touching ObjectA), which is why we do not need to worry
    // about only detecting one contact per pair of colliding objects.
    PhysicsBall* ball = nullptr;
    if(user_data_a->type == PhysicsObjectType::BALL) {
        ball = static_cast<PhysicsBall*>(user_data_a->physics_object);
    }
    if(user_data_b->type == PhysicsObjectType::BALL) {
        ball = static_cast<PhysicsBall*>(user_data_b->physics_object);
    }

    PhysicsRobot* robot = nullptr;
    if(user_data_a->type == PhysicsObjectType::ROBOT_CHICKER) {
        robot = static_cast<PhysicsRobot*>(user_data_a->physics_object);
    }
    if(user_data_b->type == PhysicsObjectType::ROBOT_CHICKER) {
        robot = static_cast<PhysicsRobot*>(user_data_b->physics_object);
    }

    if(ball && robot) {
        return std::make_pair(ball, robot);
    }

    return std::nullopt;
}

std::optional<std::pair<PhysicsBall*, PhysicsRobot*>> SimulationContactListener::isBallDribblerContact(
        PhysicsObjectUserData *user_data_a, PhysicsObjectUserData *user_data_b) {
    if(!user_data_a || !user_data_b) {
        return std::nullopt;
    }

    // NOTE: Box2D intelligently reports only one contact when 2 objects collide
    // (ie. Does not report a contact for ObjectA touching ObjectB, and another
    // contact for ObjectB touching ObjectA), which is why we do not need to worry
    // about only detecting one contact per pair of colliding objects.
    PhysicsBall* ball = nullptr;
    if(user_data_a->type == PhysicsObjectType::BALL) {
        ball = static_cast<PhysicsBall*>(user_data_a->physics_object);
    }
    if(user_data_b->type == PhysicsObjectType::BALL) {
        ball = static_cast<PhysicsBall*>(user_data_b->physics_object);
    }

    PhysicsRobot* robot = nullptr;
    if(user_data_a->type == PhysicsObjectType::ROBOT_DRIBBLER) {
        robot = static_cast<PhysicsRobot*>(user_data_a->physics_object);
    }
    if(user_data_b->type == PhysicsObjectType::ROBOT_DRIBBLER) {
        robot = static_cast<PhysicsRobot*>(user_data_b->physics_object);
    }

    if(ball && robot) {
        return std::make_pair(ball, robot);
    }

    return std::nullopt;
}

std::optional<std::pair<PhysicsBall*, PhysicsRobot*>> SimulationContactListener::isBallRobotBodyContact(
        PhysicsObjectUserData *user_data_a, PhysicsObjectUserData *user_data_b) {
    if(!user_data_a || !user_data_b) {
        return std::nullopt;
    }

    // NOTE: Box2D intelligently reports only one contact when 2 objects collide
    // (ie. Does not report a contact for ObjectA touching ObjectB, and another
    // contact for ObjectB touching ObjectA), which is why we do not need to worry
    // about only detecting one contact per pair of colliding objects.
    PhysicsBall* ball = nullptr;
    if(user_data_a->type == PhysicsObjectType::BALL) {
        ball = static_cast<PhysicsBall*>(user_data_a->physics_object);
    }
    if(user_data_b->type == PhysicsObjectType::BALL) {
        ball = static_cast<PhysicsBall*>(user_data_b->physics_object);
    }

    PhysicsRobot* robot = nullptr;
    if(user_data_a->type == PhysicsObjectType::ROBOT_BODY) {
        robot = static_cast<PhysicsRobot*>(user_data_a->physics_object);
    }
    if(user_data_b->type == PhysicsObjectType::ROBOT_BODY) {
        robot = static_cast<PhysicsRobot*>(user_data_b->physics_object);
    }

    if(ball && robot) {
        return std::make_pair(ball, robot);
    }

    return std::nullopt;
}

PhysicsBall* SimulationContactListener::isBallContact(PhysicsObjectUserData *user_data_a,
                                                      PhysicsObjectUserData *user_data_b) {
    PhysicsBall* ball = nullptr;
    if(user_data_a && user_data_a->type == PhysicsObjectType::BALL) {
        ball = static_cast<PhysicsBall*>(user_data_a->physics_object);
    }
    if(user_data_b && user_data_b->type == PhysicsObjectType::BALL) {
        ball = static_cast<PhysicsBall*>(user_data_b->physics_object);
    }

    return ball;
}


