#pragma once

#include <gtest/gtest.h>

#include "software/ai/ai.h"
#include "software/gui/visualizer/visualizer_wrapper.h"
#include "software/simulated_tests/validation/continuous_function_validator.h"
#include "software/simulated_tests/validation/function_validator.h"
#include "software/simulation/simulator.h"
#include "software/sensor_fusion/sensor_fusion.h"

/**
 * This is a test fixture designed to make it easy to write integration tests. It provides
 * an easy interface to set up robots on the field, and then validate how the world
 * changes over time during simulation. This allows us to easily write tests for
 * the AI's behaviour.
 */
class SimulatedTest : public ::testing::Test
{
public:
    explicit SimulatedTest();

   protected:
    void SetUp() override;

    /**
     * This function enables the Visualizer while a test is running, so that the test can
     * be debugged Visually. Simply call this function at the start of the test(s) you
     * want to show in the Visualizer.
     */
    void enableVisualizer();

    /**
     * Starts the simulation using the current state of the simulator, and runs
     * the given ValidationFunctions on each new state of the World. The test
     * will succeed if
     * - There are no validation_functions and all continuous_validation_functions
     *   pass for the duration of the test. In this case the test will run for the
     *   duration of the timeout.
     * - There is at least one validation_function, and all validation functions
     *   continuous and non-continuous) pass. In this case the test will run until
     *   all validation_functions have passed or the timeout is exceeded, whichever
     *   comes first
     *
     * This function will block until the test has either succeeded or encounters
     * a fatal failure.
     *
     * @param validation_functions The validation functions to validate each
     * state of the World
     * @param continuous_validation_functions The continuous validation functions
     * to validate each state of the world
     * @param timeout The maximum duration of simulated time to run the test for.
     * If the test has not passed by the time this timeout is exceeded, the test
     * will fail.
     */
    void runTest(
            const std::vector<ValidationFunction> &validation_functions,
            const std::vector<ValidationFunction> &continuous_validation_functions,
            const Duration& timeout
            );

    /**
     * Sets the state of the ball in the simulation. No more than 1 ball may exist
     * in the simulation at a time. If a ball does not already exist, a ball
     * is added with the given state. If a ball already exists, it's state is set to the
     * given state.
     *
     * @param ball_state The new ball state
     */
    void setBallState(const BallState& ball);

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
    void addFriendlyRobots(const std::vector<RobotStateWithId>& robots);
    void addEnemyRobots(const std::vector<RobotStateWithId>& robots);

    /**
     * Sets the goalie for the specified team. If this function is not called,
     * the goalie will be set to the default ID of the DynamicParameters
     *
     * @param goalie_id The ID of the robot to be goalie
     */
    void setFriendlyGoalie(RobotId goalie_id);
    void setEnemyGoalie(RobotId goalie_id);

    /**
     * Sets the play to run in the simulated test
     *
     * @param play_name The name of the play
     */
    void setPlay(const std::string& play_name);

    /**
     * Returns the field in the simulated test
     *
     * @return the field in the simulated test
     */
    Field field() const;

private:
    /**
     * A helper function that updates SensorFusion with the latest data from the Simulator
     */
    void updateSensorFusion();

    /**
     * Runs the given function validators and returns whether or not the
     * FunctionValidators have completed (Note completed does not necessarily
     * mean passed).
     *
     * @param function_validators The FunctionValidators to check
     * @param continuous_function_validators The ContinuousFunctionValidators to check
     *
     * @return true if there is at least one FunctionValidator and all FunctionValidators
     * have completed, and false otherwise
     */
    static bool validate(std::vector<FunctionValidator>& function_validators,
                         std::vector<ContinuousFunctionValidator>& continuous_function_validators);

    /**
     * Puts the current thread to sleep until the difference between the current wall time and the
     * wall_start_time is >= the difference between current_time and zero. This "synchronizes"
     * wall time with simulation time by slowing down execution if needed.
     *
     * @param wall_start_time The time at which the simulated test started, in wall time
     * @param current_time The current time in the simulated test
     */
    static void sleep(const std::chrono::steady_clock::time_point& wall_start_time, const Timestamp& current_time);

    // The simulator needs to be a pointer so that we can destroy and re-create
    // the object in the SetUp function. Because the simulator has no
    // copy assignment operator, we have to make it a dynamically-allocated
    // object so we can assign new instances to this variable
    std::unique_ptr<Simulator> simulator;
    SensorFusion sensor_fusion;
    AI ai;

    std::vector<ContinuousFunctionValidator> continuous_function_validators;
    std::vector<FunctionValidator> function_validators;

    std::shared_ptr<VisualizerWrapper> visualizer;
    // If false, runs the simulation as fast as possible.
    // If true, introduces artifical delay so that simulation
    // time passes at the same speed a real life time
    bool run_simulation_in_realtime;

    // The rate at which camera data will be simulated and given to SensorFusion.
    // Each sequential "camera frame" will be 1 / SIMULATED_CAMERA_FPS time step
    // ahead of the previous one
    static constexpr unsigned int SIMULATED_CAMERA_FPS = 60;
    // In real-life, the AI typically runs slower than we receive data. In order
    // to mimic real-life as much as possible, we define approximately how many
    // camera frames we receive per AI tick. For example, a value of 2 means
    // that we will simulate 2 time steps (2 camera frames) before we give
    // the latest data to the AI and run it.
    static constexpr unsigned int CAMERA_FRAMES_PER_AI_TICK = 2;
};
