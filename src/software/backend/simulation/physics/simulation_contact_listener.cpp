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
            SimulatorBall* ball = ball_chicker_pair->first;
            SimulatorRobot* robot = ball_chicker_pair->second;
            handleBallChickerContact(contact, ball, robot);
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
            SimulatorBall* ball = ball_dribbler_pair->first;
            SimulatorRobot* robot = ball_dribbler_pair->second;
            handleBallDribblerContact(contact, ball, robot);
        }
    }
}

std::optional<std::pair<SimulatorBall*, SimulatorRobot*>> SimulationContactListener::isBallChickerContact(PhysicsObjectUserData *user_data_a,
                                                     PhysicsObjectUserData *user_data_b)
{
    if(!user_data_a || !user_data_b) {
        return std::nullopt;
    }

    SimulatorBall* ball = nullptr;
    if(user_data_a->type == PhysicsObjectType::BALL) {
        auto physics_ball = static_cast<PhysicsBall*>(user_data_a->physics_object);
        ball = static_cast<SimulatorBall*>(physics_ball->getSimulatorBall());
    }
    if(user_data_b->type == PhysicsObjectType::BALL) {
        auto physics_ball = static_cast<PhysicsBall*>(user_data_b->physics_object);
        ball = static_cast<SimulatorBall*>(physics_ball->getSimulatorBall());
    }

    SimulatorRobot* robot = nullptr;
    if(user_data_a->type == PhysicsObjectType::ROBOT_CHICKER) {
        auto physics_robot = static_cast<PhysicsRobot*>(user_data_a->physics_object);
        robot = static_cast<SimulatorRobot*>(physics_robot->getSimulatorRobot());
    }
    if(user_data_b->type == PhysicsObjectType::ROBOT_CHICKER) {
        auto physics_robot = static_cast<PhysicsRobot*>(user_data_b->physics_object);
        robot = static_cast<SimulatorRobot*>(physics_robot->getSimulatorRobot());
    }

    if(ball && robot) {
        return std::make_pair(ball, robot);
    }

    return std::nullopt;
}

std::optional<std::pair<SimulatorBall*, SimulatorRobot*>> SimulationContactListener::isBallDribblerContact(
        PhysicsObjectUserData *user_data_a, PhysicsObjectUserData *user_data_b) {
    if(!user_data_a || !user_data_b) {
        return std::nullopt;
    }

    SimulatorBall* ball = nullptr;
    if(user_data_a->type == PhysicsObjectType::BALL) {
        auto physics_ball = static_cast<PhysicsBall*>(user_data_a->physics_object);
        ball = static_cast<SimulatorBall*>(physics_ball->getSimulatorBall());
    }
    if(user_data_b->type == PhysicsObjectType::BALL) {
        auto physics_ball = static_cast<PhysicsBall*>(user_data_b->physics_object);
        ball = static_cast<SimulatorBall*>(physics_ball->getSimulatorBall());
    }

    SimulatorRobot* robot = nullptr;
    if(user_data_a->type == PhysicsObjectType::ROBOT_DRIBBLER) {
        auto physics_robot = static_cast<PhysicsRobot*>(user_data_a->physics_object);
        robot = static_cast<SimulatorRobot*>(physics_robot->getSimulatorRobot());
    }
    if(user_data_b->type == PhysicsObjectType::ROBOT_DRIBBLER) {
        auto physics_robot = static_cast<PhysicsRobot*>(user_data_b->physics_object);
        robot = static_cast<SimulatorRobot*>(physics_robot->getSimulatorRobot());
    }

    if(ball && robot) {
        return std::make_pair(ball, robot);
    }

    return std::nullopt;
}

void SimulationContactListener::handleBallChickerContact(b2Contact *contact, SimulatorBall *ball, SimulatorRobot *robot) {
     if(auto autokick_speed_m_per_s = robot->getAutokickSpeed()) {
         Vector kick_vector = Vector::createFromAngle(Angle::fromRadians(robot->getOrientation())).normalize(autokick_speed_m_per_s.value());
         ball->applyImpulse(kick_vector);
     }else if(auto autochip_distance_m = robot->getAutochipDistance()) {
         Vector chip_vector = Vector::createFromAngle(Angle::fromRadians(robot->getOrientation())).normalize(autochip_distance_m.value());
         // TODO: chipping logic
         ball->applyImpulse(chip_vector);
     }
}

void SimulationContactListener::handleBallDribblerContact(b2Contact *contact, SimulatorBall *ball, SimulatorRobot *robot) {
    if(robot->getDribblerSpeed() > 0) {
        Vector orthogonal_force_vector = -Vector::createFromAngle(Angle::fromRadians(robot->getOrientation()));
        orthogonal_force_vector.normalize(100);
        ball->applyForce(orthogonal_force_vector);
        // TODO: centering force
    }
}


