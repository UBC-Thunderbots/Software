#include "software/gui/visualizer_wrapper.h"
#include "software/gui/drawing/world.h"

VisualizerWrapper::VisualizerWrapper(int argc, char** argv) :
      ThreadedObserver<World>(),
      ThreadedObserver<AIDrawFunction>(),
      ThreadedObserver<PlayInfo>(),
      ThreadedObserver<RobotStatus>()
{
    auto application_promise =
        std::make_shared<std::promise<std::shared_ptr<QApplication>>>();
    std::future<std::shared_ptr<QApplication>> application_future =
        application_promise->get_future();
    auto visualizer_promise =
        std::make_shared<std::promise<std::shared_ptr<Visualizer>>>();
    std::future<std::shared_ptr<Visualizer>> visualizer_future =
        visualizer_promise->get_future();
    run_visualizer_thread =
        std::thread(&VisualizerWrapper::createAndRunVisualizer, this, argc, argv,
                    application_promise, visualizer_promise);

    // We use futures and promises here to force the constructor to wait for the newly spawned
    // thread to fully create the application and visualizer objects before we take pointers to
    // them. If we do not make this guarantee, we could get invalid references to the application
    // or visualizer objects which will cause the system to fail when we try interact with them
    application = application_future.get();
    visualizer  = visualizer_future.get();
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

void VisualizerWrapper::createAndRunVisualizer(
    int argc, char** argv,
    std::shared_ptr<std::promise<std::shared_ptr<QApplication>>> application_promise_ptr,
    std::shared_ptr<std::promise<std::shared_ptr<Visualizer>>> visualizer_promise_ptr)
{
    auto app = std::make_shared<QApplication>(argc, argv);
    application_promise_ptr->set_value(app);
    auto viz = std::make_shared<Visualizer>();
    viz->show();
    visualizer_promise_ptr->set_value(viz);
    app->exec();
}

void VisualizerWrapper::onValueReceived(World world)
{
    most_recent_world_draw_function = getDrawWorldFunction(world);
    draw();
}

void VisualizerWrapper::onValueReceived(AIDrawFunction draw_function)
{
    most_recent_ai_draw_function = draw_function;
    draw();
}

void VisualizerWrapper::onValueReceived(PlayInfo play_info)
{
    most_recent_play_info = play_info;
    updatePlayInfo();
}

void VisualizerWrapper::onValueReceived(RobotStatus robot_status)
{
    // Call the Visualizer to update the Play Info in a threadsafe manner
    // See
    // https://stackoverflow.com/questions/10868946/am-i-forced-to-use-pthread-cond-broadcast-over-pthread-cond-signal-in-order-to/10882705#10882705
    QMetaObject::invokeMethod(visualizer.get(), "updateRobotStatus",
                              Qt::ConnectionType::BlockingQueuedConnection,
                              Q_ARG(RobotStatus, robot_status));
}

void VisualizerWrapper::draw()
{
    // Call the Visualizer to draw the AI in a threadsafe manner
    // See
    // https://stackoverflow.com/questions/10868946/am-i-forced-to-use-pthread-cond-broadcast-over-pthread-cond-signal-in-order-to/10882705#10882705
    QMetaObject::invokeMethod(visualizer.get(), "draw",
                              Qt::ConnectionType::BlockingQueuedConnection,
                              Q_ARG(WorldDrawFunction, most_recent_world_draw_function),
                              Q_ARG(AIDrawFunction, most_recent_ai_draw_function));
}

void VisualizerWrapper::updatePlayInfo()
{
    // Call the Visualizer to update the Play Info in a threadsafe manner
    // See
    // https://stackoverflow.com/questions/10868946/am-i-forced-to-use-pthread-cond-broadcast-over-pthread-cond-signal-in-order-to/10882705#10882705
    QMetaObject::invokeMethod(visualizer.get(), "updatePlayInfo",
                              Qt::ConnectionType::BlockingQueuedConnection,
                              Q_ARG(PlayInfo, most_recent_play_info));
}
