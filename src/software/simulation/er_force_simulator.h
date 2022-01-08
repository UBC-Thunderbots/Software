#pragma once

#include "extlibs/er_force_sim/src/amun/simulator/simulator.h"
#include "proto/defending_side_msg.pb.h"
#include "proto/messages_robocup_ssl_wrapper.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/simulation/er_force_simulator_ball.h"
#include "software/simulation/er_force_simulator_robot.h"
#include "software/simulation/firmware_object_deleter.h"
#include "software/world/field.h"
#include "software/world/team_types.h"
#include "software/world/world.h"

extern "C"
{
#include "firmware/shared/physics.h"
#include "proto/primitive.nanopb.h"
#include "proto/robot_log_msg.nanopb.h"
#include "proto/tbots_software_msgs.nanopb.h"
}

/**
 * The ErForceSimulator abstracts away the physics simulation of all objects in the world,
 * as well as the firmware simulation for the robots. This provides a simple interface
 * to setup, run, and query the current state of the simulation.
 */
class ErForceSimulator : public QObject
{
   public:
    /**
     * Creates a new Simulator. The starting state of the simulation
     * will have the given field, with no robots or ball.
     *
     * @param field The field to initialize the simulation with
     * @param robot_constants The robot constants
     * @param wheel_constants The wheel constants
     * @param simulator_config The config to fetch parameters from
     */
    explicit ErForceSimulator(const FieldType& field_type,
                              const RobotConstants_t& robot_constants,
                              const WheelConstants& wheel_constants,
                              std::shared_ptr<const SimulatorConfig> simulator_config);
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
    void addYellowRobots(const std::vector<RobotStateWithId>& robots);
    void addBlueRobots(const std::vector<RobotStateWithId>& robots);

    /**
     * Sets the primitive being simulated by the robot on the corresponding team
     * in simulation
     *
     * @param primitive_set_msg The set of primitives to run on the robot
     * @param vision_msg The vision message
     */
    void setYellowRobotPrimitiveSet(const TbotsProto::PrimitiveSet& primitive_set_msg,
                                    std::unique_ptr<TbotsProto::Vision> vision_msg);
    void setBlueRobotPrimitiveSet(const TbotsProto::PrimitiveSet& primitive_set_msg,
                                  std::unique_ptr<TbotsProto::Vision> vision_msg);

    /**
     * Advances the simulation by the given time step.
     *
     * @param time_step how much to advance the simulation by
     */
    void stepSimulation(const Duration& time_step);

    /**
     * Returns the most recent SSL Wrapper Packets
     *
     * @return vector of `SSLProto::SSL_WrapperPacket`s representing the most recent state
     * of the simulation
     */
    std::vector<SSLProto::SSL_WrapperPacket> getSSLWrapperPackets() const;

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
     * @param primitive_msg The primitive to run on the robot
     * @param simulator_robots The robots to set the primitives on
     * @param vision_msg The vision message
     */
    static void setRobotPrimitive(
        RobotId id, const TbotsProto::Primitive& primitive_msg,
        std::vector<std::shared_ptr<ErForceSimulatorRobot>>& simulator_robots,
        const TbotsProto::Vision& vision_msg);

    /**
     * Update Simulator Robot and get the latest robot control
     *
     * @param simulator_robots Vector of simulator robots
     * @param vision_msg The vision msg for this team of robots
     *
     * @return robot control
     */
    static SSLSimulationProto::RobotControl updateSimulatorRobots(
        std::vector<std::shared_ptr<ErForceSimulatorRobot>> simulator_robots,
        TbotsProto::Vision vision_msg);

    std::vector<std::shared_ptr<ErForceSimulatorRobot>> yellow_simulator_robots;
    std::vector<std::shared_ptr<ErForceSimulatorRobot>> blue_simulator_robots;
    std::unique_ptr<TbotsProto::Vision> yellow_team_vision_msg;
    std::unique_ptr<TbotsProto::Vision> blue_team_vision_msg;

    unsigned int frame_number;

    // The current time.
    Timestamp current_time;

    amun::SimulatorSetup er_force_sim_setup;
    std::unique_ptr<camun::simulator::Simulator> er_force_sim;

    RobotConstants_t robot_constants;
    WheelConstants wheel_constants;
    Field field;

    const QString CONFIG_FILE      = "simulator/2020";
    const QString CONFIG_DIRECTORY = "extlibs/er_force_sim/config/";
};
