#pragma once

#include "multithreading/threaded_observer.h"
#include "ai/world/world.h"
#include <thread>
#include <QApplication>
#include "gui/main_window.h"

class VisualizerWrapper : public ThreadedObserver<World> {
public:
    VisualizerWrapper(int argc, char **argv);
    ~VisualizerWrapper();

private:
    void createAndRunVisualizer(int argc, char**argv, std::shared_ptr<ThunderbotsVisualizer>& visualizer_ref);
    void onValueReceived(World world) override;
    void drawAI();
    World most_recent_world;
    std::thread run_visualizer_thread;
    std::shared_ptr<ThunderbotsVisualizer> visualizer;
};

