#pragma once

#include <QtWidgets/QApplication>
#include <future>
#include <thread>

#include "software/ai/world/world.h"
#include "software/gui/drawing/draw_functions.h"
#include "software/gui/widgets/main_window.h"
#include "software/multithreading/threaded_observer.h"
#include "software/ai/hl/stp/play_info.h"
#include <unordered_set>

/**
 * This class wraps our 'ThunderbotsVisualizer' object which is responsible for
 * visualizing information about our AI, and allowing users to control it.
 */
class VisualizerWrapper : public ThreadedObserver<World>,
        public ThreadedObserver<WorldDrawFunction>,
        public ThreadedObserver<AIDrawFunction>,
        public ThreadedObserver<PlayInfo>
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

    ~VisualizerWrapper() override;

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

    void onValueReceived(AIDrawFunction draw_function) override;
    void onValueReceived(WorldDrawFunction draw_function) override;
    void onValueReceived(PlayInfo play_info) override;

    /**
     * Draws all the AI information in the Visualizer. This includes visualizing the state
     * of the world as well as drawing the AI state we want to show, like planned
     * navigator paths.
     */
    void draw();
    void updatePlayInfo();

    PlayInfo most_recent_play_info;
    AIDrawFunction most_recent_ai_draw_function;
    WorldDrawFunction most_recent_world_draw_function;
    std::thread run_visualizer_thread;
    std::shared_ptr<ThunderbotsVisualizer> visualizer;
    std::shared_ptr<QApplication> application;
};
