#pragma once

#include <QtWidgets/QApplication>
#include <future>
#include <thread>

#include "software/ai/world/world.h"
#include "software/gui/drawing/typedefs.h"
#include "software/gui/widgets/main_window.h"
#include "software/multithreading/threaded_observer.h"

/**
 * This class wraps our 'ThunderbotsVisualizer' object which is responsible for
 * visualizing information about our AI, and allowing users to control it.
 */
class VisualizerWrapper : public ThreadedObserver<World>, public ThreadedObserver<DrawFunction>
{
   public:
    VisualizerWrapper() = delete;

    /**
     * Create a new Visualizer wrapper
     *
     * @param argc The number of arguments being passed
     * @param argv Keyword arguments
     */
    explicit VisualizerWrapper(int argc, char** argv);

    ~VisualizerWrapper();

   private:
    /**
     * Creates a new ThunderbotsVisualizer in a new thread and starts running it. We use
     * promises in order for this object to still get pointers to the newly created
     * QApplication and ThunderbotsVisualizer objects so we can control them. These
     * objects must be created in the new thread because the QApplication must be
     * constructed in the thread is will run in, and the ThunderbotsVisualizer must be
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
        std::shared_ptr<std::promise<std::shared_ptr<ThunderbotsVisualizer>>>
            visualizer_promise_ptr);

    void onValueReceived(World world) override;

    void onValueReceived(DrawFunction draw_function) override;

    /**
     * Draws all the AI information in the Visualizer. This includes visualizing the state
     * of the world as well as drawing the AI state we want to show, like planned
     * navigator paths.
     */
    void drawAI();

    World most_recent_world;
    std::thread run_visualizer_thread;
    std::shared_ptr<ThunderbotsVisualizer> visualizer;
    std::shared_ptr<QApplication> application;
};
