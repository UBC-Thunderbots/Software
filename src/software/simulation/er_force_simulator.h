#pragma once

#include "extlibs/er_force_sim/src/amun/simulator/simulator.h"
#include "extlibs/er_force_sim/src/core/timer.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "shared/proto/tbots_software_msgs.pb.h"
#include "software/proto/defending_side_msg.pb.h"
#include "software/proto/messages_robocup_ssl_wrapper.pb.h"
#include "software/simulation/er_force_simulator_ball.h"
#include "software/simulation/er_force_simulator_robot.h"
#include "software/simulation/firmware_object_deleter.h"
#include "software/simulation/physics/physics_world.h"
#include "software/world/field.h"
#include "software/world/team_types.h"
#include "software/world/world.h"

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
class ErForceSimulator : public QObject
{
    Q_OBJECT
   public slots:
    void setWrapperPacket(const QByteArray& data, qint64 time, QString sender);

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
    explicit ErForceSimulator(const Field& field, const RobotConstants_t& robot_constants,
                              const WheelConstants& wheel_constants,
                              std::shared_ptr<const SimulatorConfig> simulator_config,
                              const Duration& physics_time_step = Duration::fromSeconds(
                                  DEFAULT_PHYSICS_TIME_STEP_SECONDS));
    ErForceSimulator() = delete;
    ~ErForceSimulator();

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
    void setYellowRobotPrimitiveSet(const TbotsProto_PrimitiveSet& primitive_set_msg,
                                    const TbotsProto::Vision& vision_msg);
    void setBlueRobotPrimitiveSet(const TbotsProto_PrimitiveSet& primitive_set_msg,
                                  const TbotsProto::Vision& vision_msg);

    /**
     * Advances the simulation by the given time step. This will simulate
     * one "camera frame" of data and increase the camera_frame value by 1.
     *
     * @param time_step how much to advance the simulation by
     */
    void stepSimulation(const Duration& time_step);

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
     * Sets the primitive being simulated by the robot on the corresponding team
     * in simulation
     *
     * @param id The id of the robot to set the primitive for
     * @param primitive_msg The primitive to run on the robot
     * @param vision_msg The vision message
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
     * @param vision_msg The vision message
     */
    static void setRobotPrimitive(
        RobotId id, const TbotsProto_Primitive& primitive_msg,
        std::map<std::shared_ptr<ErForceSimulatorRobot>,
                 std::shared_ptr<FirmwareWorld_t>>& simulator_robots,
        std::shared_ptr<ErForceSimulatorBall> simulator_ball,
        const TbotsProto::Vision& vision_msg);

    PhysicsWorld physics_world;
    std::shared_ptr<ErForceSimulatorBall> simulator_ball;
    std::map<std::shared_ptr<ErForceSimulatorRobot>, std::shared_ptr<FirmwareWorld_t>>
        yellow_simulator_robots;
    std::map<std::shared_ptr<ErForceSimulatorRobot>, std::shared_ptr<FirmwareWorld_t>>
        blue_simulator_robots;
    TbotsProto::Vision yellow_team_vision_msg;
    TbotsProto::Vision blue_team_vision_msg;

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
    camun::simulator::Simulator* er_force_sim;
    SSLProto::SSL_WrapperPacket wrapper_packet;
};
