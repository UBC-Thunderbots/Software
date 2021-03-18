#pragma once

class SslSimulationProtocolManager
{
  public:
    /**
     * Sets up the initial simulation with robots, ball state, etc
     */
    SslSimulationProtocolManager::setupInitialSimulationState()

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
     * Sets the state of the ball in the simulation. No more than 1 ball may exist
     * in the simulation at a time. If a ball does not already exist, a ball
     * is added with the given state. If a ball already exists, its state is set to the
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

  private:
    /**
     * Sets the primitive being simulated by the robot on the corresponding team
     * in simulation
     *
     * @param id The id of the robot to set the primitive for
     * @param primitive_msg The primitive to run on the robot
     */
    void setYellowRobotPrimitive(RobotId id, const TbotsProto_Primitive& primitive_msg);
    void setBlueRobotPrimitive(RobotId id, const TbotsProto_Primitive& primitive_msg);

    /**
     * Runs the primitives on each robot once, and sends the latest robot commands
     */
    void tick();

    /**
     * Set up networking
     */
    void setupNetworking();

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
    void setBlueTeamDefendingSide(const DefendingSideProto& defending_side_proto)

    std::shared_ptr<SslSimulatorBall> simulator_ball;
    std::map<std::shared_ptr<SimulatorRobot>, std::shared_ptr<FirmwareWorld_t>>
        yellow_simulator_robots;
    std::map<std::shared_ptr<SimulatorRobot>, std::shared_ptr<FirmwareWorld_t>>
        blue_simulator_robots;
    FieldSide yellow_team_defending_side;
    FieldSide blue_team_defending_side;

    std::unique_ptr<ThreadedProtoMulticastListener<TbotsProto::PrimitiveSet>>
        yellow_team_primitive_listener, blue_team_primitive_listener;
    SslSimulationProtocolNetworkModule sslSimulationProtocolNetworkModule;
};