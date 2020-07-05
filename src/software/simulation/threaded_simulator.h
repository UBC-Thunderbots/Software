#pragma once

#include <atomic>
#include <thread>

#include "software/proto/messages_robocup_ssl_wrapper.pb.h"
#include "software/simulation/simulator.h"

/**
 * A wrapper for Simulator that runs in a separate thread with a callback registry
 */
class ThreadedSimulator
{
   public:
    /**
     * Creates a new ThreadedSimulator. The starting state of the simulation
     * will have the given field, with no robots or ball.
     *
     * @param field The field to initialize the simulation with
     */
    explicit ThreadedSimulator(const Field& field);
    ~ThreadedSimulator();

    /**
     * Registers the given callback function. This callback function will be
     * called each time the simulation updates and a new SSL_WrapperPacket
     * is generated.
     *
     * Note: This function is threadsafe
     *
     * @param callback The callback function to register
     */
    void registerOnSSLWrapperPacketReadyCallback(
        const std::function<void(SSL_WrapperPacket)>& callback);

    /**
     * Starts running the simulator in a new thread. This is a non-blocking call.
     * If the simulator is already running, this function does nothing.
     *
     * Note: This function is threadsafe
     */
    void startSimulation();

    /**
     * Stops the simulation if it is running. If the simulation is not already
     * running, this function does nothing.
     *
     * Note: This function is threadsafe
     */
    void stopSimulation();

    /**
     * Sets the state of the ball in the simulation. No more than 1 ball may exist
     * in the simulation at a time. If a ball does not already exist, a ball
     * is added with the given state. If a ball already exists, it's state is set to the
     * given state.
     *
     * Note: This function is threadsafe.
     *
     * @param ball_state The new ball state
     */
    void setBallState(const BallState& ball_state);

    /**
     * Removes the ball from the physics world. If a ball does not already exist,
     * this has no effect.
     *
     * Note: This function is threadsafe.
     */
    void removeBall();

    /**
     * Adds robots to the specified team with the given initial states.
     *
     * Note: These functions are threadsafe.
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
     * Note: These functions are threadsafe.
     *
     * @param primitives The primitives to simulate
     */
    void setYellowRobotPrimitives(ConstPrimitiveVectorPtr primitives);
    void setBlueRobotPrimitives(ConstPrimitiveVectorPtr primitives);

    /**
     * Sets the primitive being simulated by the robot in simulation
     *
     * @param id The id of the robot to set the primitive for
     * @param primitive_type The primitive to set
     * @param params The parameters for the specified primitive
     */
    void setYellowRobotPrimitive(RobotId id, unsigned int primitive_index,
                                 const primitive_params_t& params);
    void setBlueRobotPrimitive(RobotId id, unsigned int primitive_index,
                               const primitive_params_t& params);

   private:
    /**
     * The function that runs inside the simulation thread, handling
     * updating the simulation and calling the callback functions.
     *
     * Note that because this function puts the simulation thread to
     * sleep in order to simulate real-time simulation (ie. simulation
     * time passes at the same speed as wall time), this means we will
     * always run at realtime or slightly slower because of timing jitter
     * due to the thread scheduler.
     */
    void runSimulationLoop();

    std::vector<std::function<void(SSL_WrapperPacket)>> ssl_wrapper_packet_callbacks;
    std::mutex callback_mutex;

    Simulator simulator;
    std::mutex simulator_mutex;
    std::thread simulation_thread;
    bool simulation_thread_started;
    std::mutex simulation_thread_started_mutex;
    std::atomic_bool stopping_simulation;

    // 60HZ is approximately the framerate of the real-life cameras
    static constexpr double TIME_STEP_SECONDS = 1.0 / 60.0;
};
