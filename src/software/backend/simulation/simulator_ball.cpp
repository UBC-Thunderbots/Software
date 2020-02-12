#include "software/backend/simulation/simulator_ball.h"
#include "software/logger/init.h"

SimulatorBall::SimulatorBall(std::weak_ptr<PhysicsBall> physics_ball) :physics_ball(physics_ball){
}

Point SimulatorBall::position() const {
    if (auto ball = physics_ball.lock()) {
        return ball->getBallWithTimestamp(Timestamp::fromSeconds(0)).position();
    }
    LOG(WARNING) << "SimulatorBall being used with invalid PhysicsBall" << std::endl;
    return Point(0, 0);
}



Vector SimulatorBall::velocity() const {
    if (auto ball = physics_ball.lock()) {
        return ball->getBallWithTimestamp(Timestamp::fromSeconds(0)).velocity();
    }
    LOG(WARNING) << "SimulatorBall being used with invalid PhysicsBall" << std::endl;
    return Vector(0, 0);
}
