#pragma once

#include "extlibs/er_force_sim/src/amun/simulator/simulator.h"
#include "proto/robot_status_msg.pb.h"
#include "proto/ssl_vision_wrapper.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "software/jetson_nano/primitive_executor.h"
#include "software/world/field.h"
#include "software/world/team_types.h"
#include "software/world/world.h"


/**
 * The ErForceSimulator abstracts away the physics simulation of all objects in the world,
 * as well as the firmware simulation for the robots. This provides a simple interface
 * to setup, run, and query the current state of the simulation.
 */
class ErForceSimulator
{
   public:
    /**
     * Creates a new Simulator. The starting state of the simulation
     * will have the given field, with no robots or ball.
     *
     * @param field_type The field type
     * @param robot_constants The robot constants
     */
    explicit ErForceSimulator(const TbotsProto::FieldType& field_type,
                              const RobotConstants_t& robot_constants);
    ErForceSimulator()  = delete;
    ~ErForceSimulator() = default;

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
    void setYellowRobots(const std::vector<RobotStateWithId>& robots);
    void setBlueRobots(const std::vector<RobotStateWithId>& robots);
    void setRobots(const std::vector<RobotStateWithId>& robots,
                   gameController::Team team);
    void setRobots(const google::protobuf::Map<uint32_t, TbotsProto::RobotState>& robots,
                   gameController::Team side);

    /**
     * Set the world state from a WorldState proto in the simulation.
     *
     * @param world_state The new WorldState
     */
    void setWorldState(const TbotsProto::WorldState& world_state);

    /**
     * Sets the primitive being simulated by the robot on the corresponding team
     * in simulation
     *
     * @param primitive_set_msg The set of primitives to run on the robot
     * @param world_msg The world message
     */
    void setYellowRobotPrimitiveSet(const TbotsProto::PrimitiveSet& primitive_set_msg,
                                    std::unique_ptr<TbotsProto::World> world_msg);
    void setBlueRobotPrimitiveSet(const TbotsProto::PrimitiveSet& primitive_set_msg,
                                  std::unique_ptr<TbotsProto::World> world_msg);

    /**
     * Advances the simulation by the given time step.
     *
     * @param time_step how much to advance the simulation by
     */
    void stepSimulation(const Duration& time_step);

    /**
     * Gets the blue and yellow robot statuses
     *
     * @return a vector of robot statuses from either blue or yellow robots
     */
    std::vector<TbotsProto::RobotStatus> getBlueRobotStatuses() const;
    std::vector<TbotsProto::RobotStatus> getYellowRobotStatuses() const;

    /**
     * Returns the most recent SSL Wrapper Packets
     *
     * @return vector of `SSLProto::SSL_WrapperPacket`s representing the most recent state
     * of the simulation
     */
    std::vector<SSLProto::SSL_WrapperPacket> getSSLWrapperPackets() const;

    /**
     * Returns the current Simulator State
     */
    world::SimulatorState getSimulatorState() const;

    /**
     * Returns the field in the simulation
     *
     * @return the field in the simulation
     */
    Field getField() const;

    /**
     * Returns the current time in the simulation
     *
     * @return the current time in the simulation
     */
    Timestamp getTimestamp() const;

    /**
     * Resets the current time to 0
     */
    void resetCurrentTime();

   private:
    /**
     * Sets the primitive being simulated by the robot in simulation
     *
     * @param id The id of the robot to set the primitive for
     * @param primitive_set_msg The primitive to run on the robot
     * @param robot_primitive_executor_map The robot primitive executors to send the
     * primitive set to
     * @param world_msg The world message
     * @param local_velocity The local velocity
     */
    static void setRobotPrimitive(
        RobotId id, const TbotsProto::PrimitiveSet& primitive_set_msg,
        std::unordered_map<unsigned int, std::shared_ptr<PrimitiveExecutor>>&
            robot_primitive_executor_map,
        const TbotsProto::World& world_msg, Vector local_velocity);

    /**
     * Gets a map from robot id to local velocity from repeated sim robots
     *
     * @param repeated sim robots
     *
     * @return a map from robot id to local velocity
     */
    static std::map<RobotId, Vector> getRobotIdToLocalVelocityMap(
        const google::protobuf::RepeatedPtrField<world::SimRobot>& sim_robots);

    /**
     * Update Simulator Robot and get the latest robot control
     *
     * @param robot_primitive_executor_map Map of robot IDs to the robot's primitive
     * executor
     * @param world_msg The world msg for this team of robots
     *
     * @return robot control
     */
    SSLSimulationProto::RobotControl updateSimulatorRobots(
        std::unordered_map<unsigned int, std::shared_ptr<PrimitiveExecutor>>&
            robot_primitive_executor_map,
        const TbotsProto::World& world_msg);

    // Map of Robot id to Primitive Executor
    std::unordered_map<unsigned int, std::shared_ptr<PrimitiveExecutor>>
        yellow_primitive_executor_map;
    std::unordered_map<unsigned int, std::shared_ptr<PrimitiveExecutor>>
        blue_primitive_executor_map;
    std::unique_ptr<TbotsProto::World> yellow_team_world_msg;
    std::unique_ptr<TbotsProto::World> blue_team_world_msg;

    static constexpr double primitive_executor_time_step = 1.0 / 60.0;
    unsigned int frame_number;

    // The current time.
    Timestamp current_time;

    amun::SimulatorSetup er_force_sim_setup;
    std::unique_ptr<camun::simulator::Simulator> er_force_sim;

    RobotConstants_t robot_constants;
    Field field;

    std::optional<RobotId> blue_robot_with_ball;
    std::optional<RobotId> yellow_robot_with_ball;

    const QString CONFIG_FILE      = "simulator/2020";
    const QString CONFIG_DIRECTORY = "extlibs/er_force_sim/config/";
};
