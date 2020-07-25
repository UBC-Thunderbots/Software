#include "software/simulated_tests/simulated_test_simulator.h"

#include <pb_decode.h>

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

void SimulatedTestSimulator::setYellowRobotSerializedPrimitiveSet(
    std::vector<uint8_t> serialized_primitive_set_msg)
{
    PrimitiveSetMsg primitive_set_msg = PrimitiveSetMsg_init_zero;

    pb_istream_t pb_in_stream =
        pb_istream_from_buffer(static_cast<uint8_t*>(serialized_primitive_set_msg.data()),
                               serialized_primitive_set_msg.size());
    if (!pb_decode(&pb_in_stream, PrimitiveSetMsg_fields, &primitive_set_msg))
    {
        throw std::runtime_error("Failed to decode serialized primitive to NanoPb");
    }

    simulator_static->setYellowRobotPrimitiveSet(primitive_set_msg);
}
