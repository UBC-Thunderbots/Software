#include "software/simulated_tests/serialized_to_nano_pb_simulator_adapter.h"

#include <pb_decode.h>

#include "software/simulation/simulator.h"

SerializedToNanoPbSimulatorAdapter::SerializedToNanoPbSimulatorAdapter(const Field& field)
    : simulator(std::make_shared<Simulator>(field))
{
}

void SerializedToNanoPbSimulatorAdapter::setBallState(const BallState& ball_state)
{
    simulator->setBallState(ball_state);
}

void SerializedToNanoPbSimulatorAdapter::addYellowRobots(
    const std::vector<RobotStateWithId>& robots)
{
    simulator->addYellowRobots(robots);
}

void SerializedToNanoPbSimulatorAdapter::addBlueRobots(
    const std::vector<RobotStateWithId>& robots)
{
    simulator->addBlueRobots(robots);
}

Field SerializedToNanoPbSimulatorAdapter::getField() const
{
    return simulator->getField();
}

std::unique_ptr<SSL_WrapperPacket>
SerializedToNanoPbSimulatorAdapter::getSSLWrapperPacket() const
{
    return simulator->getSSLWrapperPacket();
}

Timestamp SerializedToNanoPbSimulatorAdapter::getTimestamp() const
{
    return simulator->getTimestamp();
}

void SerializedToNanoPbSimulatorAdapter::stepSimulation(const Duration& time_step)
{
    simulator->stepSimulation(time_step);
}

void SerializedToNanoPbSimulatorAdapter::setYellowRobotSerializedPrimitiveSet(
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

    simulator->setYellowRobotPrimitiveSet(primitive_set_msg);
}
