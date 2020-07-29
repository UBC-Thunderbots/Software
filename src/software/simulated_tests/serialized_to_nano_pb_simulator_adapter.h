#pragma once

#include <memory>
#include <vector>

#include "software/proto/messages_robocup_ssl_wrapper.pb.h"
#include "software/time/timestamp.h"
#include "software/world/ball_state.h"
#include "software/world/field.h"
#include "software/world/robot_state.h"

// We forward declare the Simulator because the simulator depends on NanoPb
// PrimitiveSetMsg, which we will not expose to users of this class by only including
// simulator.h in serialized_to_nano_pb_simulator_adapter.cpp
class Simulator;
/**
 * This class adapts the Simulator's NanoPb protobuf interface to serialized protobufs, so
 * that users of this class do not have to depend on NanoPb protobufs
 */
class SerializedToNanoPbSimulatorAdapter
{
   public:
    SerializedToNanoPbSimulatorAdapter() = delete;

    /**
     * Creates a new SerializedToNanoPbSimulatorAdapter. The starting state of the
     * simulation will have the given field, with no robots or ball.
     *
     * @param field The field to initialize the simulation with
     */
    explicit SerializedToNanoPbSimulatorAdapter(const Field& field);

    /**
     * Sets the state of the ball in the simulation. No more than 1 ball may exist
     * in the simulation at a time. If a ball does not already exist, a ball
     * is added with the given state. If a ball already exists, it's state is set to the
     * given state.
     *
     * @param ball_state The new ball state
     */
    void setBallState(const BallState& ball_state);

    /**
     * Adds robots to the specified team with the given initial states.
     *
     * @pre The robot IDs must not be duplicated and must not match the ID
     * of any robot already on the specified team.
     *
     * @throws runtime_error if any of the given robot ids are duplicated, or a
     * robot already exists on the specified team with one of the new IDs
     *
     * @param robots the robots to add
     */
    void addYellowRobots(const std::vector<RobotStateWithId>& robots);
    void addBlueRobots(const std::vector<RobotStateWithId>& robots);


    /**
     * Returns the field in the simulation
     *
     * @return the field in the simulation
     */
    Field getField() const;

    /**
     * Returns an SSL_WrapperPacket representing the most recent state
     * of the simulation
     *
     * @return an SSL_WrapperPacket representing the most recent state
     * of the simulation
     */
    std::unique_ptr<SSL_WrapperPacket> getSSLWrapperPacket() const;

    /**
     * Returns the current time in the simulation
     *
     * @return the current time in the simulation
     */
    Timestamp getTimestamp() const;

    /**
     * Advances the simulation by the given time step. This will simulate
     * one "camera frame" of data and increase the camera_frame value by 1.
     *
     * @param time_step how much to advance the simulation by
     */
    void stepSimulation(const Duration& time_step);

    /**
     * Sets the primitives on the corresponding team in simulation
     *
     * @param serialized_primitive_set_msg the serialized proto of the PrimitiveSetMsg to
     * set
     */
    void setYellowRobotSerializedPrimitiveSet(
        std::vector<uint8_t> serialized_primitive_set_msg);

   private:
    std::shared_ptr<Simulator> simulator;
};
