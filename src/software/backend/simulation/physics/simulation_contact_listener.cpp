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
            PhysicsBall* ball = ball_dribbler_pair->first;
            PhysicsRobot* robot = ball_dribbler_pair->second;
        }
    }
}

std::optional<std::pair<PhysicsBall*, PhysicsRobot*>> SimulationContactListener::isBallChickerContact(PhysicsObjectUserData *user_data_a,
                                                     PhysicsObjectUserData *user_data_b)
{
    if(!user_data_a || !user_data_b) {
        return std::nullopt;
    }

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

void SimulationContactListener::handleBallChickerContact(b2Contact *contact, PhysicsBall *ball, PhysicsRobot *robot) {
     if(robot->autokick_enabled) {
         get kick vector
         ball->applyImpulse(kick vector);
     }else if(robot->autochip_enabled) {
         get chip vector
         ball->applyImpulse(chip vector);
     }

}

void SimulationContactListener::handleBallDribblerContact(b2Contact *contact, PhysicsBall *ball, PhysicsRobot *robot) {

}


