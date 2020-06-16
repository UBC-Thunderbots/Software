#pragma once

#include "software/primitive/primitive.h"
#include "software/proto/messages_robocup_ssl_wrapper.pb.h"
#include "software/simulation/physics/physics_world.h"
#include "software/simulation/simulator_ball.h"
#include "software/simulation/simulator_robot.h"
#include "software/world/world.h"

/**
 * Because the FirmwareWorld_t struct is defined in the .c file (rather than the .h file),
 * C++ considers it an incomplete type and is unable to use it with smart pointers
 * because it doesn't know the size of the object. Therefore we need to create our own
 * "Deleter" class we can provide to the smart pointers to handle that instead.
 *
 * See https://en.cppreference.com/w/cpp/memory/unique_ptr/unique_ptr for more info and
 * examples
 */
struct FirmwareWorldDeleter
{
    void operator()(FirmwareWorld_t* firmware_world) const
    {
        FirmwareRobot_t* firmware_robot = app_firmware_world_getRobot(firmware_world);

        Wheel_t* firmware_robot_front_left_wheel =
            app_firmware_robot_getFrontLeftWheel(firmware_robot);
        app_wheel_destroy(firmware_robot_front_left_wheel);
        Wheel_t* firmware_robot_back_left_wheel =
            app_firmware_robot_getBackLeftWheel(firmware_robot);
        app_wheel_destroy(firmware_robot_back_left_wheel);
        Wheel_t* firmware_robot_back_right_wheel =
            app_firmware_robot_getBackRightWheel(firmware_robot);
        app_wheel_destroy(firmware_robot_back_right_wheel);
        Wheel_t* firmware_robot_front_right_wheel =
            app_firmware_robot_getFrontRightWheel(firmware_robot);
        app_wheel_destroy(firmware_robot_front_right_wheel);

        Chicker_t* firmware_robot_chicker = app_firmware_robot_getChicker(firmware_robot);
        app_chicker_destroy(firmware_robot_chicker);

        Dribbler_t* firmware_robot_dribbler =
            app_firmware_robot_getDribbler(firmware_robot);
        app_dribbler_destroy(firmware_robot_dribbler);

        app_firmware_robot_destroy(firmware_robot);

        FirmwareBall_t* firmware_ball = app_firmware_world_getBall(firmware_world);
        app_firmware_ball_destroy(firmware_ball);

        app_firmware_world_destroy(firmware_world);
    };
};

/**
 * The Simulator abstracts away the physics simulation of all objects in the world,
 * as well as the firmware simulation for the robots. This provides a simple interface
 * to setup, run, and query the current state of the simulation.
 */
class Simulator
{
   public:
    /**
     * Creates a new Simulator. The starting state of the simulation
     * will have the given field, with no robots or ball.
     *
     * @param field The field to initialize the simulation with
     */
    explicit Simulator(const Field& field);
    Simulator() = delete;

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
     * Removes the ball from the physics world. If a ball does not already exist,
     * this has no effect.
     */
    void removeBall();

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
     * Sets the primitives being simulated by the robots in simulation
     *
     * @param primitives The primitives to simulate
     */
    void setYellowRobotPrimitives(ConstPrimitiveVectorPtr primitives);
    void setBlueRobotPrimitives(ConstPrimitiveVectorPtr primitives);

    /**
     * Advances the simulation by the given time step. This will simulate
     * physics and primitives
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

    std::unique_ptr<SSL_WrapperPacket> getSSLWrapperPacket() const;

   private:
    /**
     * Updates the given simulator_robots to contain and control the given physics_robots
     *
     * @param physics_robots The physics robots to add to the simulator robots
     * @param simulator_robots The simulator robots to add the physics robots to
     */
    static void updateSimulatorRobots(
        const std::vector<std::weak_ptr<PhysicsRobot>>& physics_robots,
        std::map<std::shared_ptr<SimulatorRobot>, std::shared_ptr<FirmwareWorld_t>>&
            simulator_robots);

    /**
     * Sets the given primitives on the given simulator robots
     *
     * @param primitives The primitives to set
     * @param simulator_robots The robots to set the primitives on
     * @param simulator_ball The simulator ball to use in the primitives
     */
    static void setRobotPrimitives(
        ConstPrimitiveVectorPtr primitives,
        std::map<std::shared_ptr<SimulatorRobot>, std::shared_ptr<FirmwareWorld_t>>&
            simulator_robots,
        const std::shared_ptr<SimulatorBall>& simulator_ball);

    /**
     * Returns the encoded primitive parameters for the given Primitive
     *
     * @param primitive The Primitive to get the parameters for
     *
     * @return The encoded primitive parameters for the given Primitive
     */
    static primitive_params_t getPrimitiveParams(
        const std::unique_ptr<Primitive>& primitive);

    /**
     * Returns the primitive index for the given Primitive
     *
     * @param primitive The Primitive to get the index for
     *
     * @return The index for the given Primitive
     */
    static unsigned int getPrimitiveIndex(const std::unique_ptr<Primitive>& primitive);

    PhysicsWorld physics_world;
    std::shared_ptr<SimulatorBall> simulator_ball;
    std::map<std::shared_ptr<SimulatorRobot>, std::shared_ptr<FirmwareWorld_t>>
        yellow_simulator_robots;
    std::map<std::shared_ptr<SimulatorRobot>, std::shared_ptr<FirmwareWorld_t>>
        blue_simulator_robots;

    unsigned int frame_number;

    // The camera ID of all SSLDetectionFrames published by the simulator.
    // This simulates having a single camera that can see the entire field
    static constexpr unsigned int CAMERA_ID            = 0;
    static constexpr float FIELD_LINE_THICKNESS_METRES = 0.01;
};
