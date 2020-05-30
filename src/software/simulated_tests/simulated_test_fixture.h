#pragma once

#include <gtest/gtest.h>

#include "software/ai/ai_wrapper.h"
#include "software/backend/simulator_backend.h"
#include "software/gui/visualizer/visualizer_wrapper.h"
#include "software/simulated_tests/validation/world_state_validator.h"

/**
 * This is a test fixture designed to make it easy to write integration tests with the
 * SimulatorBackend. It uses the SimulatorBackend to simulate and publish the World, and
 * the WorldStateValidator to make assertions about the world. This allows us to easily
 * write tests for the AI's behaviour.
 */
class SimulatedTest : public ::testing::Test
{
   protected:
    /**
     * The setup function that will run before each test case
     */
    void SetUp() override;

    /**
     * The teardown function that will run after each test case
     */
    void TearDown() override;

    /**
     * This function enables the Visualizer while a test is running, so that the test can
     * be debugged Visually. Simply call this function at the start of the test(s) you
     * want to show in the Visualizer. Make sure to run the test targets with 'bazel run'
     * instead of 'bazel test' or the Visualizer won't work.
     */
    void enableVisualizer();

    std::shared_ptr<SimulatorBackend> backend;
    std::shared_ptr<VisualizerWrapper> visualizer;
    std::shared_ptr<AIWrapper> ai_wrapper;
    std::shared_ptr<WorldStateValidator> world_state_validator;
};
