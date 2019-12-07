#pragma once

#include <QtCore/QGenericArgument>
#include <QtWidgets/QApplication>
#include <thread>
#include <future>
#include <atomic>

#include "software/ai/hl/stp/play_info.h"
#include "software/backend/robot_status.h"
#include "software/multithreading/threaded_observer.h"
#include "software/multithreading/thread_safe_buffer.h"
#include "software/visualizer/drawing/draw_functions.h"
#include "software/visualizer/widgets/visualizer.h"
#include "software/world/world.h"

/**
 * This class wraps our Visualizer object which is responsible for
 * visualizing information about our AI, and allowing users to control it.
 */
class VisualizerWrapper : public ThreadedObserver<World>,
                          public ThreadedObserver<AIDrawFunction>,
                          public ThreadedObserver<PlayInfo>,
                          public ThreadedObserver<RobotStatus>
{
   public:
    VisualizerWrapper() = delete;

    /**
     * Create a new Visualizer wrapper. The argc and argv arguments are required
     * to create a QApplication
     *
     * @param argc The number of arguments being passed
     * @param argv Keyword arguments
     */
    explicit VisualizerWrapper(int argc, char** argv);

    ~VisualizerWrapper() override;

    /**
     * Returns a shared_ptr to a promise that can be waited on, and that will
     * be notified once the Visualizer has shut down
     *
     * @return a shared_ptr to a promise that can be waited on, and that will
     * be notified once the Visualizer has been shut down
     */
    std::shared_ptr<std::promise<void>> getTerminationPromise();

   private:
    /**
     * Creates a new Visualizer in a new thread and starts running it. These
     * objects must be created in the new thread because the QApplication must be
     * constructed in the thread is will run in, and the Visualizer must be
     * created in the same context as the QApplication (which in this case is the new
     * thread).
     *
     * @param argc The number of arguments being passed
     * @param argv Keyword arguments for the Visualizer QApplication
     */
    void createAndRunVisualizer(int argc, char** argv);

    void onValueReceived(World world) override;
    void onValueReceived(AIDrawFunction draw_function) override;
    void onValueReceived(PlayInfo play_info) override;
    void onValueReceived(RobotStatus robot_status) override;

    std::thread run_visualizer_thread;
    std::shared_ptr<std::promise<void>> termination_promise_ptr;

    // Buffers that are shared with the instance of the Visualizer so that data can
    // be passed safely
    std::shared_ptr<ThreadSafeBuffer<WorldDrawFunction>> world_draw_functions_buffer;
    std::shared_ptr<ThreadSafeBuffer<AIDrawFunction>> ai_draw_functions_buffer;
    std::shared_ptr<ThreadSafeBuffer<PlayInfo>> play_info_buffer;
    std::shared_ptr<ThreadSafeBuffer<RobotStatus>> robot_status_buffer;

    std::atomic_bool application_shutting_down;
};
