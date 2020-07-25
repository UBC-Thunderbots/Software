#include "software/simulated_tests/simulated_test_simulator.h"

#include "software/simulation/simulator.h"

// We use a static variable here to hide simulator.h's NanoPb PrimitiveSetMsg from users
// of this class
static std::shared_ptr<Simulator> simulator_static;

SimulatedTestSimulator::SimulatedTestSimulator(const Field& field)
{
    simulator_static = std::make_shared<Simulator>(field);
}

void SimulatedTestSimulator::setBallState(const BallState& ball_state)
{
    simulator_static->setBallState(ball_state);
}

void SimulatedTestSimulator::addYellowRobots(const std::vector<RobotStateWithId>& robots)
{
    simulator_static->addYellowRobots(robots);
}

void SimulatedTestSimulator::addBlueRobots(const std::vector<RobotStateWithId>& robots)
{
    simulator_static->addBlueRobots(robots);
}

Field SimulatedTestSimulator::getField() const
{
    return simulator_static->getField();
}

std::unique_ptr<SSL_WrapperPacket> SimulatedTestSimulator::getSSLWrapperPacket() const
{
    return simulator_static->getSSLWrapperPacket();
}

Timestamp SimulatedTestSimulator::getTimestamp() const
{
    return simulator_static->getTimestamp();
}

void SimulatedTestSimulator::stepSimulation(const Duration& time_step)
{
    simulator_static->stepSimulation(time_step);
}

void SimulatedTestSimulator::setYellowRobotPrimitive(RobotId id,
                                                     const PrimitiveMsg& primitive_msg)
{
    simulator_static->setYellowRobotPrimitive(id, primitive_msg);
}
