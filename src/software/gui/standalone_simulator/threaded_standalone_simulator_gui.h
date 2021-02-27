#pragma once

#include <atomic>
#include <future>
#include <thread>

#include "software/simulation/standalone_simulator.h"

/**
 * This class wraps a StandaloneSimulatorGUI object so it can run independently
 * in its own thread. This way the GUI can be rendered and handle events separately
 * without affecting the main application.
 */
class ThreadedStandaloneSimulatorGUI
{
   public:
    /**
     * Creates a new ThreadedStandaloneSimulatorGUI
     *
     * @pre simulator must point to a valid StandaloneSimulator (ie. must not be null)
     *
     * @param simulator The StandaloneSimulator this GUI will control
     */
    explicit ThreadedStandaloneSimulatorGUI(
        std::shared_ptr<StandaloneSimulator> simulator, 
        std::shared_ptr<SimulatorConfig> mutable_simulator_config,
        std::shared_ptr<StandaloneSimulatorConfig> mutable_standalone_simulator_config
        );

    ~ThreadedStandaloneSimulatorGUI();

    /**
     * Returns a shared_ptr to a promise that can be waited on, and that will
     * be notified once the GUI has shut down
     *
     * @return a shared_ptr to a promise that can be waited on, and that will
     * be notified once the GUI has been shut down
     */
    std::shared_ptr<std::promise<void>> getTerminationPromise();

   private:
    /**
     * Creates a StandaloneSimulatorGUI in a new thread and starts running it. These
     * objects must be created in the new thread because the QApplication must be
     * constructed in the thread it will run in, and the StandaloneSimulatorGUI must be
     * created in the same context as the QApplication (which in this case is the new
     * thread).
     *
     * @pre simulator must point to a valid StandaloneSimulator (ie. must not be null)
     *
     * @param simulator The StandaloneSimulator this GUI will control
     */
    void createAndRunStandaloneSimulatorGUI(
        std::shared_ptr<StandaloneSimulator> simulator);

    std::thread run_standalone_simulator_gui_thread;
    std::shared_ptr<SimulatorConfig> mutable_simulator_config;
    std::shared_ptr<StandaloneSimulatorConfig> mutable_standalone_simulator_config;
    std::shared_ptr<std::promise<void>> termination_promise_ptr;

    std::atomic_bool application_shutting_down;
};
