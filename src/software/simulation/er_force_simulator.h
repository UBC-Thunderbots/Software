#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/proto/defending_side_msg.pb.h"
#include "software/proto/messages_robocup_ssl_wrapper.pb.h"
#include "software/simulation/firmware_object_deleter.h"
#include "software/simulation/physics/physics_world.h"
#include "software/simulation/physics_simulator_ball.h"
#include "software/simulation/physics_simulator_robot.h"
#include "software/world/field.h"
#include "software/world/team_types.h"
#include "software/world/world.h"
#include "src/amun/simulator/simulator.h"
#include "src/core/timer.h"

extern "C"
{
#include "firmware/shared/physics.h"
#include "shared/proto/primitive.nanopb.h"
#include "shared/proto/tbots_software_msgs.nanopb.h"
}

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
     * @param field The field to initialize the simulation with
     * @param simulator_config The config to fetch parameters from
     * @param physics_time_step The time step used to simulated physics
     * and robot primitives.
     */
    explicit ErForceSimulator(const Field& field,
                              std::shared_ptr<const SimulatorConfig> simulator_config,
                              const Duration& physics_time_step = Duration::fromSeconds(
                                  DEFAULT_PHYSICS_TIME_STEP_SECONDS));
    ErForceSimulator() = delete;

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
     * Adds a robots to the specified team at the given position. The robot will
     * automatically be given a valid ID.
     *
     * @param position the position at which to add the robot
     */
    void addYellowRobot(const Point& position);
    void addBlueRobot(const Point& position);

    /**
     * Sets the primitive being simulated by the robot on the corresponding team
     * in simulation
     *
     * @param primitive_set_msg The set of primitives to run on the robot
     * @param vision_msg The vision message
     */
    void setYellowRobotPrimitiveSet(const TbotsProto_PrimitiveSet& primitive_set_msg,
                                    const TbotsProto_Vision vision_msg);
    void setBlueRobotPrimitiveSet(const TbotsProto_PrimitiveSet& primitive_set_msg,
                                  const TbotsProto_Vision vision_msg);

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
    void setBlueTeamDefendingSide(const DefendingSideProto& defending_side_proto);

    /**
     * Advances the simulation by the given time step. This will simulate
     * one "camera frame" of data and increase the camera_frame value by 1.
     *
     * @param time_step how much to advance the simulation by
     */
    void stepSimulation(const Duration& time_step);

    /**
     * Returns the current state of the world in the simulation
     *
     * @return the current state of the world in the simulation
     */
    World getWorld() const;

    /**
     * Returns an SSLProto::SSL_WrapperPacket representing the most recent state
     * of the simulation
     *
     * @return an SSLProto::SSL_WrapperPacket representing the most recent state
     * of the simulation
     */
    std::unique_ptr<SSLProto::SSL_WrapperPacket> getSSLWrapperPacket() const;

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
     * Removes the given PhysicsRobot from the PhysicsWorld, if it exists.
     *
     * @param robot The robot to be removed
     */
    void removeRobot(std::weak_ptr<PhysicsRobot> robot);

    /**
     * Resets the current firmware time to 0
     */
    static void resetCurrentFirmwareTime();

   private:
    /**
     * Get the current time.
     *
     * This is passed into a `FirmwareWorld`, which requires that it is static (as it
     * is C code). This will just return `current_firmware_time`, which should be updated
     * to the actual current time before ticking any firmware.
     *
     * @return The value of `current_firmware_time`, in seconds.
     */
    static float getCurrentFirmwareTimeSeconds();

    /**
     * Updates the given simulator_robots to contain and control the given physics_robots
     *
     * @param physics_robots The physics robots to add to the simulator robots
     * @param simulator_robots The simulator robots to add the physics robots to
     * @param team_colour The color of the team this robot is on
     */
    static void updateSimulatorRobots(
        const std::vector<std::weak_ptr<PhysicsRobot>>& physics_robots,
        std::map<std::shared_ptr<PhysicsSimulatorRobot>,
                 std::shared_ptr<FirmwareWorld_t>>& simulator_robots,
        TeamColour team_colour);

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
     * Sets the primitive being simulated by the robot in simulation
     *
     * @param id The id of the robot to set the primitive for
     * @param primitive_msg The primitive to run on the robot
     * @param simulator_robots The robots to set the primitives on
     * @param simulator_ball The simulator ball to use in the primitives
     * @param defending_side The side of the field the robot is defending
     */
    static void setRobotPrimitive(
        RobotId id, const TbotsProto_Primitive& primitive_msg,
        std::map<std::shared_ptr<PhysicsSimulatorRobot>,
                 std::shared_ptr<FirmwareWorld_t>>& simulator_robots,
        const std::shared_ptr<PhysicsSimulatorBall>& simulator_ball,
        FieldSide defending_side);

    PhysicsWorld physics_world;
    std::shared_ptr<PhysicsSimulatorBall> simulator_ball;
    std::map<std::shared_ptr<PhysicsSimulatorRobot>, std::shared_ptr<FirmwareWorld_t>>
        yellow_simulator_robots;
    std::map<std::shared_ptr<PhysicsSimulatorRobot>, std::shared_ptr<FirmwareWorld_t>>
        blue_simulator_robots;
    FieldSide yellow_team_defending_side;
    FieldSide blue_team_defending_side;

    unsigned int frame_number;

    // The time step used to simulate physics and primitives
    const Duration physics_time_step;

    // The camera ID of all SSLDetectionFrames published by the simulator.
    // This simulates having a single camera that can see the entire field
    static constexpr unsigned int CAMERA_ID            = 0;
    static constexpr float FIELD_LINE_THICKNESS_METRES = 0.01f;
    // We reuse the firmware tick rate to mimic real firmware
    static constexpr double DEFAULT_PHYSICS_TIME_STEP_SECONDS = 1.0 / CONTROL_LOOP_HZ;

    // The current time. This is static so that it may be used by the firmware,
    // and so must be set before each firmware tick
    static Timestamp current_firmware_time;

    Timer er_force_sim_timer;
    amun::SimulatorSetup er_force_sim_setup;
    camun::simulator::Simulator er_force_sim;
};
