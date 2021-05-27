#pragma once

#include "shared/test_util/tbots_gtest_main.h"
#include "software/ai/hl/stp/play/halt_play.h"
#include "software/gui/full_system/threaded_full_system_gui.h"
#include "software/proto/logging/proto_logger.h"
#include "software/sensor_fusion/sensor_fusion.h"
#include "software/simulated_tests/validation/non_terminating_function_validator.h"
#include "software/simulated_tests/validation/terminating_function_validator.h"
#include "software/simulation/simulator.h"

/**
 * This is a test fixture designed to make it easy to write integration tests. It provides
 * an easy interface to set up robots on the field, and then validate how the world
 * changes over time during simulation. This allows us to easily write tests for
 * the AI's behaviour.
 */
class SimulatedTestFixture : public ::testing::Test
{
   public:
    explicit SimulatedTestFixture();

   protected:
    void SetUp() override;

    /**
     * This function enables the FullSystemGUI while a test is running, so that the test
     * can be debugged Visually. Simply call this function at the start of the test(s) you
     * want to show in the FullSystemGUI.
     */
    void enableVisualizer();

    /**
     * Creates a directory to output logs to in the directory at the
     * TEST_UNDECLARED_OUTPUTS_DIR Bazel environment variable, and sets up some
     * ProtoLoggers to log unfiltered and filtered data.
     *
     * See
     * https://docs.bazel.build/versions/master/test-encyclopedia.html#initial-conditions
     * for an explanation of all the environment variables that Bazel passes to tests
     */
    void setupReplayLogging();

    /**
     * Starts the simulation using the current state of the simulator, and runs
     * the given ValidationFunctions on each new state of the World. The test
     * will succeed if
     * - There are no terminating validation functions and all non-terminating
     *   validation functions pass for the duration of the test. In this case
     *   the test will run for the duration of the timeout.
     * - There is at least one terminating validation function, and all
     *   validation functions (terminating and non-terminating) pass. In this case
     *   the test will run until all terminating validation functions have
     *   completed or the timeout is exceeded, whichever comes first
     *
     * This function will block until the test has either succeeded or encounters
     * a fatal failure.
     *
     * @pre The robot IDs must not be duplicated and must not match the ID
     * of any robot already on the specified team.
     *
     * @throws runtime_error if any of the given robot ids are duplicated, or a
     * robot already exists on the specified team with one of the new IDs
     *
     * @param field The field to run the test on
     * @param ball_state The ball state to run the test with
     * @param friendly_robots The friendly robot states with ID to run the test with
     * @param enemy_robots The enemy robot states with ID to run the test with
     * @param terminating_validation_functions The terminating validation functions
     * to check during the test
     * @param non_terminating_validation_functions The non-terminating validation
     * functions to check during the test
     * @param timeout The maximum duration of simulated time to run the test for.
     * If the test has not passed by the time this timeout is exceeded, the test
     * will fail.
     */
    void runTest(
        const Field &field, const BallState &ball,
        const std::vector<RobotStateWithId> &friendly_robots,
        const std::vector<RobotStateWithId> &enemy_robots,
        const std::vector<ValidationFunction> &terminating_validation_functions,
        const std::vector<ValidationFunction> &non_terminating_validation_functions,
        const Duration &timeout);

    /**
     * Registers a new tick time for calculating tick time statistics
     *
     * @param tick_time_ms The tick time in milliseconds
     */
    void registerTickTime(double tick_time_ms);

    // The dynamic params being used in the tests
    std::shared_ptr<ThunderbotsConfig> mutable_thunderbots_config;
    std::shared_ptr<const ThunderbotsConfig> thunderbots_config;

   private:
    /**
     * Runs one tick of the test and checks if the validation function is done
     *
     * @param simulation_time_step time step for stepping the simulation
     * @param ai_time_step minimum time for one tick of AI
     * @param world the shared_ptr to the world that is updated by this function
     * @param simulator The simulator to tick test on
     *
     * @return if validation functions are done
     */
    bool tickTest(Duration simulation_time_step, Duration ai_time_step,
                  std::shared_ptr<World> world, std::shared_ptr<Simulator> simulator);

    /**
     * A helper function that updates SensorFusion with the latest data from the Simulator
     *
     * @param simulator The simulator to update sensor fusion with
     */
    void updateSensorFusion(std::shared_ptr<Simulator> simulator);

    /**
     * Updates primitives in the simulator based on the new world
     *
     * @param world to update primitives with
     * @param simulator_to_update The simulator to update
     */
    virtual void updatePrimitives(const World &world,
                                  std::shared_ptr<Simulator> simulator_to_update) = 0;

    /**
     * Gets play info for displaying on the FullSystemGUI
     *
     * @return play info to display, if any
     */
    virtual std::optional<PlayInfo> getPlayInfo() = 0;

    /**
     * Gets draw functions for visualizing on the FullSystemGUI
     *
     * @return draw functions to draw
     */
    virtual AIDrawFunction getDrawFunctions() = 0;

    /**
     * Runs the given function validators and returns whether or not the
     * FunctionValidators have completed (Note: completed does not necessarily
     * mean passed).
     *
     * @param terminating_function_validators The TerminatingFunctionValidators to check
     * @param non_terminating_function_validators The NonTerminatingFunctionValidators to
     * check
     *
     * @return true if there is at least one TerminatingFunctionValidator and all
     * TerminatingFunctionValidators have completed, and false otherwise
     */
    static bool validateAndCheckCompletion(
        std::vector<TerminatingFunctionValidator> &terminating_function_validators,
        std::vector<NonTerminatingFunctionValidator>
            &non_terminating_function_validators);

    /**
     * Puts the current thread to sleep such that each simulation step will take
     * the desired amount of real-world "wall" time.
     *
     * @param wall_start_time The time at which the most recent simulation step started,
     * in wall time
     * @param desired_wall_tick_time How long each simulation step should take
     * in wall-clock time
     */
    static void sleep(const std::chrono::steady_clock::time_point &wall_start_time,
                      const Duration &desired_wall_tick_time);

    // The simulator needs to be a pointer so that we can destroy and re-create
    // the object in the SetUp function. Because the simulator has no
    // copy assignment operator, we have to make it a dynamically-allocated
    // object so we can assign new instances to this variable
    std::shared_ptr<Simulator> simulator;
    // The SensorFusion being tested and used in simulation
    SensorFusion sensor_fusion;

    // whether we should log the filtered and unfiltered world states as replay logs
    // this will only be set to true if the environment variable
    // TEST_UNDECLARED_OUTPUTS_DIR is set, usually by running as a Bazel test
    bool should_log_replay;
    // ProtoLoggers for the simulator and SensorFusion, respectively
    std::shared_ptr<ProtoLogger<SensorProto>> simulator_sensorproto_logger;
    std::shared_ptr<ProtoLogger<SSLProto::SSL_WrapperPacket>> sensorfusion_wrapper_logger;

    std::vector<NonTerminatingFunctionValidator> non_terminating_function_validators;
    std::vector<TerminatingFunctionValidator> terminating_function_validators;

    std::shared_ptr<ThreadedFullSystemGUI> full_system_gui;
    // If false, runs the simulation as fast as possible.
    // If true, introduces artificial delay so that simulation
    // time passes at the same speed a real life time
    bool run_simulation_in_realtime;

    // These variables track tick time statistics
    // Total duration of all ticks registered
    double total_tick_duration;
    // The max tick duration registered
    double max_tick_duration;
    // The min tick duration registered
    double min_tick_duration;
    // Total number of ticks registered
    unsigned int tick_count;

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
