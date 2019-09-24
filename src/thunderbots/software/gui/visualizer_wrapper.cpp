#include "software/gui/visualizer_wrapper.h"

VisualizerWrapper::VisualizerWrapper(int argc, char** argv) : ThreadedObserver<World>()
{
    auto application_promise =
        std::make_shared<std::promise<std::shared_ptr<QApplication>>>();
    std::future<std::shared_ptr<QApplication>> application_future =
        application_promise->get_future();
    auto visualizer_promise =
        std::make_shared<std::promise<std::shared_ptr<ThunderbotsVisualizer>>>();
    std::future<std::shared_ptr<ThunderbotsVisualizer>> visualizer_future =
        visualizer_promise->get_future();
    run_visualizer_thread =
        std::thread(&VisualizerWrapper::createAndRunVisualizer, this, argc, argv,
                    application_promise, visualizer_promise);
    application = application_future.get();
    std::cout << "got application future" << std::endl;
    visualizer = visualizer_future.get();
}

VisualizerWrapper::~VisualizerWrapper()
{
    // Call the Application in a threadsafe manner
    // See
    // https://stackoverflow.com/questions/10868946/am-i-forced-to-use-pthread-cond-broadcast-over-pthread-cond-signal-in-order-to/10882705#10882705
    QMetaObject::invokeMethod(application.get(), "quit",
                              Qt::ConnectionType::BlockingQueuedConnection);
    run_visualizer_thread.join();
}

void VisualizerWrapper::onValueReceived(World world)
{
    most_recent_world = world;
//    drawAI();
}

void VisualizerWrapper::onValueReceived(DrawFunction draw_function) {
    std::cout << "\nFOOBAR\n" << std::endl;
    most_recent_draw_function = draw_function;
    drawAITest();
}

void VisualizerWrapper::drawAI()
{
    // Call the ThunderbotsVisualizer to draw the AI in a threadsafe manner
    // See
    // https://stackoverflow.com/questions/10868946/am-i-forced-to-use-pthread-cond-broadcast-over-pthread-cond-signal-in-order-to/10882705#10882705
    QMetaObject::invokeMethod(visualizer.get(), "drawAI",
                              Qt::ConnectionType::BlockingQueuedConnection,
                              Q_ARG(World, most_recent_world));
}

void VisualizerWrapper::drawAITest()
{
    QMetaObject::invokeMethod(visualizer.get(), "drawAITest",
                              Qt::ConnectionType::BlockingQueuedConnection,
                              Q_ARG(DrawFunction, most_recent_draw_function));
}

void VisualizerWrapper::createAndRunVisualizer(
    int argc, char** argv,
    std::shared_ptr<std::promise<std::shared_ptr<QApplication>>> application_promise_ptr,
    std::shared_ptr<std::promise<std::shared_ptr<ThunderbotsVisualizer>>>
        visualizer_promise_ptr)
{
    auto app = std::make_shared<QApplication>(argc, argv);
    application_promise_ptr->set_value(app);
    auto viz = std::make_shared<ThunderbotsVisualizer>();
    viz->show();
    visualizer_promise_ptr->set_value(viz);
    app->exec();
}
