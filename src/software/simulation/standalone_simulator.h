#pragma once

#include "proto/defending_side_msg.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/networking/threaded_proto_udp_listener.h"
#include "software/networking/threaded_proto_udp_sender.h"
#include "software/simulation/threaded_simulator.h"

/**
 * This class abstracts all simulation and networking operations for
 * a StandaloneSimulator. The StandaloneSimulator can be run as a separate
 * application on a computer or network and be interacted with by up to
 * 2 instances of an AI.
 */
class StandaloneSimulator
{
   public:
    enum class SimulationMode
    {
        PLAY,
        PAUSE,
        SLOW_MOTION
    };

    /**
     * Creates a new StandaloneSimulator, and starts the simulation.
     *
     * @param standalone_simulator_config The config for the StandaloneSimulator
     * @param simulator_config The config for the Simulator
     * @param field The field to simulate
     * @param robot_constants The robot constants
     * @param wheel_constants The wheel constants
     */
    explicit StandaloneSimulator(
        std::shared_ptr<StandaloneSimulatorConfig> standalone_simulator_config,
        std::shared_ptr<SimulatorConfig> simulator_config, const Field& field,
        const RobotConstants_t& robot_constants, const WheelConstants& wheel_constants);
    StandaloneSimulator() = delete;

    /**
     * Registers the given callback function. This callback function will be
     * called each time the simulation updates and a new SSLProto::SSL_WrapperPacket
     * is generated.
     *
     * @param callback The callback function to register
     */
    void registerOnSSLWrapperPacketReadyCallback(
        const std::function<void(SSLProto::SSL_WrapperPacket)>& callback);

    /**
     * Adds robots to predefined locations on the field
     *
     * @param num_robots How many robots to setup
     */
    void setupInitialSimulationState(unsigned num_robots);

    SSLProto::SSL_WrapperPacket getSSLWrapperPacket() const;

    /**
     * Starts the simulation. If the simulator is already running, this
     * function does nothing.
     */
    void startSimulation();

    /**
     * Stops the simulation. If the simulator is already stopped, this
     * function does nothing.
     */
    void stopSimulation();

    /**
     * Sets the slow motion multiplier for the simulation. Larger values
     * cause the simulation to run in slow motion. For example, a value
     * of 2.0 causes the simulation to run 2x slower.
     *
     * @pre multiplier is >= 1.0
     *
     * @param multiplier The slow motion multiplier
     */
    void setSlowMotionMultiplier(double multiplier);

    /**
     * Resets the slow motion multiplier value to let the simulation
     * run in real-time speed.
     */
    void resetSlowMotionMultiplier();

    /**
     * Sets the state of the ball in the simulation. No more than 1 ball may exist
     * in the simulation at a time. If a ball does not already exist, a ball
     * is added with the given state. If a ball already exists, it's state is set to the
     * given state.
     *
     * @param state The new ball state
     */
    void setBallState(const BallState& state);

    /**
     * Returns the PhysicsRobot at the given position. This function accounts
     * for robot radius, so a robot will be returned if the given position is
     * within the robot's radius from its position.
     *
     * @param position The position at which to check for a robot
     *
     * @return a weak_ptr to the PhysicsRobot at the given position if one exists,
     * otherwise returns an empty pointer
     */
    std::weak_ptr<PhysicsRobot> getRobotAtPosition(const Point& position);

    /**
     * Adds a robots to the specified team at the given position. The robot will
     * automatically be given a valid ID.
     *
     * @param position the position at which to add the robot
     */
    void addYellowRobot(const Point& position);
    void addBlueRobot(const Point& position);

    /**
     * Removes the given PhysicsRobot from the PhysicsWorld, if it exists.
     *
     * @param robot The robot to be removed
     */
    void removeRobot(std::weak_ptr<PhysicsRobot> robot);

    // This is a somewhat arbitrary value that results in slow motion
    // simulation looking appropriately / usefully slow
    static constexpr double DEFAULT_SLOW_MOTION_MULTIPLIER = 14.0;

    /**
     * Returns the robot constants for this robot state
     *
     * @return the robot constants for all robots in this simulator
     */
    const RobotConstants_t& getRobotConstants() const;

   private:
    /**
     * Sets the primitives being simulated by the robots on the respective team
     *
     * @param primitive_set_msg The set of primitives to run on the respective team
     */
    void setYellowRobotPrimitives(const TbotsProto::PrimitiveSet& primitive_set_msg);
    void setBlueRobotPrimitives(const TbotsProto::PrimitiveSet& primitive_set_msg);

    /**
     * Sets which side of the field the corresponding team is defending.
     *
     * This will flip robot and ball coordinates an applicable in order to present
     * the firmware being simulated with data that matches our coordinate convention. See
     * https://github.com/UBC-Thunderbots/Software/blob/master/docs/software-architecture-and-design.md#coordinates
     * for more information about our coordinate conventions.
     *
     * @param defending_side_proto The side to defend
     */
    void setYellowTeamDefendingSide(const DefendingSideProto& defending_side_proto);
    void setBlueTeamDefendingSide(const DefendingSideProto& defending_side_protoj);

    /**
     * A helper function that sets up all networking functionality
     *
     * @param blue_team_channel The blue team channel
     * @param yellow_team_channel The yellow team channel
     * @param network_interface The network interface to network over
     * @param vision_port The port to receive vision
     * @param ip_address The ip address to access vision
     */
    void setupNetworking(int blue_team_channel, int yellow_team_channel,
                         std::string network_interface, int vision_port,
                         std::string vision_ip_address);

    std::shared_ptr<const StandaloneSimulatorConfig> standalone_simulator_config;
    std::unique_ptr<ThreadedProtoUdpListener<TbotsProto::PrimitiveSet>>
        yellow_team_primitive_listener, blue_team_primitive_listener;
    std::unique_ptr<ThreadedProtoUdpSender<SSLProto::SSL_WrapperPacket>>
        wrapper_packet_sender;
    std::unique_ptr<ThreadedProtoUdpListener<DefendingSideProto>>
        yellow_team_side_listener, blue_team_side_listener;
    ThreadedSimulator simulator;

    SSLProto::SSL_WrapperPacket most_recent_ssl_wrapper_packet;
    mutable std::mutex most_recent_ssl_wrapper_packet_mutex;
    RobotConstants_t robot_constants;
};
