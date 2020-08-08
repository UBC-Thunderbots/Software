#pragma once

#include <atomic>
#include <thread>

#include "software/proto/messages_robocup_ssl_wrapper.pb.h"
#include "software/simulation/simulator.h"

extern "C"
{
#include "shared/proto/primitive.nanopb.h"
}

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
     * @param ball_restitution The restitution for ball collisions
     * @param ball_linear_damping The damping on the ball's linear motion
     */
    explicit ThreadedSimulator(const Field& field, double ball_restitution = 1.0,
                               double ball_linear_damping = 0.0);
    ~ThreadedSimulator();

    /**
     * Registers the given callback function. This callback function will be
     * called each time the simulation updates and a new SSLProto::SSL_WrapperPacket
     * is generated.
     *
     * Note: This function is threadsafe
     *
     * @param callback The callback function to register
     */
    void registerOnSSLWrapperPacketReadyCallback(
        const std::function<void(SSLProto::SSL_WrapperPacket)>& callback);

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
     * Sets the slow motion multiplier for the simulation. Larger values
     * cause the simulation to run in slow motion. For example, a value
     * of 2.0 causes the simulation to run 2x slower.
     *
     * Note: This function is threadsafe
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
     * @param primitive_msg The primitive to run on the robot
     */
    void setYellowRobotPrimitive(RobotId id, const PrimitiveMsg& primitive_msg);
    void setBlueRobotPrimitive(RobotId id, const PrimitiveMsg& primitive_msg);

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

    /**
     * A helper function to update the callback functions with the latest
     * data from the simulator
     */
    void updateCallbacks();

    std::vector<std::function<void(SSLProto::SSL_WrapperPacket)>>
        ssl_wrapper_packet_callbacks;
    std::mutex callback_mutex;

    Simulator simulator;
    std::mutex simulator_mutex;
    std::thread simulation_thread;
    bool simulation_thread_started;
    std::mutex simulation_thread_started_mutex;
    std::atomic_bool stopping_simulation;

    std::atomic<double> slow_motion_multiplier = 1.0;

    // 60HZ is approximately the framerate of the real-life cameras
    static constexpr double TIME_STEP_SECONDS = 1.0 / 60.0;
};
