#pragma once

#include <gtest/gtest.h>

//#include "software/ai/ai_wrapper.h"
#include "software/ai/ai.h"
//#include "software/backend/simulator_backend.h"
#include "software/gui/visualizer/visualizer_wrapper.h"
//#include "software/simulated_tests/validation/world_state_validator.h"
#include "software/simulated_tests/validation/continuous_function_validator.h"
#include "software/simulated_tests/validation/function_validator.h"
#include "software/simulation/simulator.h"
#include "software/sensor_fusion/sensor_fusion.h"
#include <memory>

/**
 * This is a test fixture designed to make it easy to write integration tests with the
 * SimulatorBackend. It uses the SimulatorBackend to simulate and publish the World, and
 * the WorldStateValidator to make assertions about the world. This allows us to easily
 * write tests for the AI's behaviour.
 */
class SimulatedTest : public ::testing::Test
{
public:
    explicit SimulatedTest();

    void setBallState(const BallState& ball);
    void addFriendlyRobots(const std::vector<RobotStateWithId>& robots);
    void addEnemyRobots(const std::vector<RobotStateWithId>& robots);
    void setFriendlyGoalie(RobotId goalie_id);
    void setEnemyGoalie(RobotId goalie_id);
    void setPlay(const std::string& play_name);

   protected:
    /**
     * The setup function that will run before each test case
     */
    void SetUp() override;

    /**
     * This function enables the Visualizer while a test is running, so that the test can
     * be debugged Visually. Simply call this function at the start of the test(s) you
     * want to show in the Visualizer. Make sure to run the test targets with 'bazel run'
     * instead of 'bazel test' or the Visualizer won't work.
     */
    void enableVisualizer();

    void runTest(
            const std::vector<ValidationFunction> &validation_functions,
            const std::vector<ValidationFunction> &continuous_validation_functions,
            const Duration& timeout
            );

private:


    // True means stop the test, false means keep going
    static bool validateWorld(std::vector<FunctionValidator>& function_validators,
                              std::vector<ContinuousFunctionValidator>& continuous_function_validators);

    std::optional<World> getSensorFusionWorld();

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

    // If false, runs the simulation as fast as possible.
    // If true, introduces artifical delay so that simulation
    // time passes at the same speed a real life time
    bool run_simulation_in_realtime;
    std::shared_ptr<VisualizerWrapper> visualizer;
};
