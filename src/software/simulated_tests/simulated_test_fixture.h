#pragma once

#include <gtest/gtest.h>

#include "software/backend/simulator_backend.h"
#include "software/gui/visualizer_wrapper.h"


class SimulatedTest : public ::testing::Test
{
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

    std::shared_ptr<SimulatorBackend> backend;
    std::shared_ptr<VisualizerWrapper> visualizer;
};
