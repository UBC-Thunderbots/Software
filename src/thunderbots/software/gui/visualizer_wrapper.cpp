#include "gui/visualizer_wrapper.h"
#include <QApplication>
#include "software/gui/main_window.h"

VisualizerWrapper::VisualizerWrapper(int argc, char **argv) : ThreadedObserver<World>() {
    // TODO: Is there a way to make sure passing by reference is safe?
    run_visualizer_thread = std::thread(&VisualizerWrapper::createAndRunVisualizer, this, argc, argv, std::ref(visualizer));
}

VisualizerWrapper::~VisualizerWrapper() {
    run_visualizer_thread.join();
}

void VisualizerWrapper::onValueReceived(World world) {
    most_recent_world = world;
    drawAI();
}

void VisualizerWrapper::drawAI() {
    if(visualizer) {
        std::cout << "invoking method" << std::endl;
        QMetaObject::invokeMethod(visualizer.get(), "drawAI", Qt::ConnectionType::BlockingQueuedConnection, Q_ARG(World, most_recent_world));
    }
}

void VisualizerWrapper::createAndRunVisualizer(int argc, char** argv, std::shared_ptr<ThunderbotsVisualizer>& visualizer_ref) {
    // TODO: Figure out how to handle signals. How does closing the visualizer shut down
    // AI, and how does shutting down AI close the visualizer
    QApplication application(argc, argv);
    visualizer_ref = std::make_shared<ThunderbotsVisualizer>();
    visualizer_ref->show();
    application.exec();
}
