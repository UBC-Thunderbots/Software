#pragma once

#include <QtCore/QGenericArgument>
#include <QtWidgets/QApplication>
#include <future>
#include <mutex>
#include <thread>

#include "software/ai/hl/stp/play_info.h"
#include "software/backend/robot_status.h"
#include "software/gui/drawing/draw_functions.h"
#include "software/gui/widgets/visualizer.h"
#include "software/multithreading/threaded_observer.h"
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
     * Creates a new Visualizer in a new thread and starts running it. We use
     * promises in order for this object to still get pointers to the newly created
     * QApplication and Visualizer objects so we can control them. These
     * objects must be created in the new thread because the QApplication must be
     * constructed in the thread is will run in, and the Visualizer must be
     * created in the same context as the QApplication (which in this case is the new
     * thread).
     *
     * @param argc The number of arguments being passed
     * @param argv Keyword arguments for the Visualizer QApplication
     * @param application_promise_ptr A shared_ptr to a QApplication promise that will be
     * set by the new thread
     * @param visualizer_promise_ptr A shared_ptr to a ThunderbotsVisualizer promise that
     * will be set by the new thread
     */
    void createAndRunVisualizer(
        int argc, char** argv,
        std::shared_ptr<std::promise<std::shared_ptr<QApplication>>>
            application_promise_ptr,
        std::shared_ptr<std::promise<std::shared_ptr<Visualizer>>> visualizer_promise_ptr,
        std::shared_ptr<std::promise<void>> termination_promise_ptr);

    void onValueReceived(World world) override;
    void onValueReceived(AIDrawFunction draw_function) override;
    void onValueReceived(PlayInfo play_info) override;
    void onValueReceived(RobotStatus robot_status) override;

    /**
     * Draws all the AI information in the Visualizer. This includes visualizing the state
     * of the world as well as drawing the AI state we want to show, like planned
     * navigator paths.
     */
    void draw();

    /**
     * Updates the PlayInfo being displayed in the Visualizer. This shows what Play and
     * Tactics the AI is using.
     */
    void updatePlayInfo();

    std::mutex world_lock;
    std::mutex ai_lock;

    PlayInfo most_recent_play_info;
    WorldDrawFunction most_recent_world_draw_function;
    AIDrawFunction most_recent_ai_draw_function;

    std::thread run_visualizer_thread;
    std::shared_ptr<std::promise<void>> termination_promise_ptr;

    std::shared_ptr<Visualizer> visualizer;
    std::shared_ptr<QApplication> application;
};
