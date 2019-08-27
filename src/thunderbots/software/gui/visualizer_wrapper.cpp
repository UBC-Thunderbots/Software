#include "gui/visualizer_wrapper.h"
#include <QApplication>
#include "software/gui/main_window.h"

VisualizerWrapper::VisualizerWrapper(int argc, char **argv) : ThreadedObserver<World>(), application(argc, argv) {
//    QApplication app(argc, argv);
//    application = std::make_shared<QApplication>(app);
//    application = std::make_shared<QApplication>(argc, argv);
    visualizer.show();
    run_visualizer_thread = std::thread(&VisualizerWrapper::createAndRunVisualizer, this);
}

VisualizerWrapper::~VisualizerWrapper() {
    run_visualizer_thread.join();
}

void VisualizerWrapper::onValueReceived(World world) {
    std::cout << "Visualizer received world" << std::endl;
    most_recent_world = world;
    drawAI();
}

void VisualizerWrapper::drawAI() {
    visualizer.drawAI(most_recent_world);
}

void VisualizerWrapper::createAndRunVisualizer() {
    // TODO: Figure out how to handle signals. How does closing the visualizer shut down
    // AI, and how does shutting down AI close the visualizer
    application.exec();
}
