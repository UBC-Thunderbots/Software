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

//        std::cout << "some contact started" << std::endl;
        if (auto ball_chicker_pair = isBallChickerContact(user_data_a, user_data_b))
        {
            std::cout << "ball chicker contact started" << std::endl;
            PhysicsBall* ball = ball_chicker_pair->first;
            PhysicsRobot* robot = ball_chicker_pair->second;
            for(auto contact_callback : robot->getChickerBallContactCallbacks()) {
                contact_callback(robot, ball);
            }
        }
    }
}

void SimulationContactListener::PreSolve(b2Contact *contact, const b2Manifold *oldManifold) {
    if (contact->IsTouching())
    {
//        auto fixture_a = contact->GetFixtureA();
//        auto fixture_b = contact->GetFixtureB();
//
//        PhysicsObjectUserData *user_data_a =
//                static_cast<PhysicsObjectUserData *>(fixture_a->GetUserData());
//        PhysicsObjectUserData *user_data_b =
//                static_cast<PhysicsObjectUserData *>(fixture_b->GetUserData());
//
//        if(auto ball_dribbler_pair = isBallDribblerContact(user_data_a, user_data_b))
//        {
//            // TODO: disable collision
//            PhysicsBall* ball = ball_dribbler_pair->first;
//            PhysicsRobot* robot = ball_dribbler_pair->second;
//            for(auto contact_callback : robot->getDribblerBallContactCallbacks()) {
//                contact_callback(robot, ball);
//            }
//        }
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

        if (auto ball_chicker_pair = isBallChickerContact(user_data_a, user_data_b))
        {
            std::cout << "ball chicker contact ended" << std::endl;
//            PhysicsBall* ball = ball_chicker_pair->first;
//            PhysicsRobot* robot = ball_chicker_pair->second;
//            for(auto contact_callback : robot->getChickerBallContactCallbacks()) {
//                contact_callback(robot, ball);
//            }
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

//void SimulationContactListener::handleBallChickerContact(b2Contact *contact, SimulatorBall *ball, SimulatorRobot *robot) {
//     if(auto autokick_speed_m_per_s = robot->getAutokickSpeed()) {
//         Vector kick_vector = Vector::createFromAngle(Angle::fromRadians(robot->getOrientation()));
//         // Figure out how much impulse to apply to change the speed of the ball by the autokick_speed
//         double change_in_momentum = ball->getMassKg() * autokick_speed_m_per_s.value();
//         kick_vector.normalize(change_in_momentum);
//         ball->applyImpulse(kick_vector);
//     }else if(auto autochip_distance_m = robot->getAutochipDistance()) {
//         Vector chip_vector = Vector::createFromAngle(Angle::fromRadians(robot->getOrientation()));
//         // TODO: chipping logic
//         ball->applyImpulse(chip_vector);
//     }
//}
//
//void SimulationContactListener::handleBallDribblerContact(b2Contact *contact, SimulatorBall *ball, SimulatorRobot *robot) {
//    if(auto autokick_speed_m_per_s = robot->getAutokickSpeed()) {
//        Vector kick_vector = Vector::createFromAngle(Angle::fromRadians(robot->getOrientation()));
//        // Figure out how much impulse to apply to change the speed of the ball by the autokick_speed
//        double change_in_momentum = ball->getMassKg() * autokick_speed_m_per_s.value();
//        kick_vector.normalize(change_in_momentum);
//        ball->applyImpulse(kick_vector);
//    }else if(auto autochip_distance_m = robot->getAutochipDistance()) {
//        Vector chip_vector = Vector::createFromAngle(Angle::fromRadians(robot->getOrientation()));
//        // TODO: chipping logic
//        ball->applyImpulse(chip_vector);
//    }
//
////    if(robot->getDribblerSpeed() > 0) {
////        Vector orthogonal_force_vector = -Vector::createFromAngle(Angle::fromRadians(robot->getOrientation()));
////        orthogonal_force_vector.normalize(100);
////        ball->applyForce(orthogonal_force_vector);
////        // TODO: centering force
////    }
//}
//
//
