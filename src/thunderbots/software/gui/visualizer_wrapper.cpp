#include "software/gui/visualizer_wrapper.h"
#include "software/gui/drawing/world.h"
#include <chrono>

VisualizerWrapper::VisualizerWrapper(int argc, char** argv) :
            ThreadedObserver<World>(),
            ThreadedObserver<WorldDrawFunction>(),
            ThreadedObserver<AIDrawFunction>(),
            ThreadedObserver<PlayInfo>(),
            ThreadedObserver<RobotStatus>()
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
    visualizer = visualizer_future.get();

    last_status_message_update_timestamp = std::chrono::steady_clock::now();
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
    most_recent_world_draw_function = getDrawWorldFunction(world);
    draw();
}

void VisualizerWrapper::onValueReceived(RobotStatus robot_status) {
//    std::cout << "got robot status" << std::endl;
    auto current_timestamp = std::chrono::steady_clock::now();
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(current_timestamp - last_status_message_update_timestamp);
    last_status_message_update_timestamp = current_timestamp;
    for(const auto& message : robot_status.robot_messages) {
        auto iter = status_messages.find(message);
        if(iter == status_messages.end()) {
            status_messages.insert(std::make_pair(message, Duration::fromSeconds(0)));
        }else {
            iter->second = iter->second + Duration::fromMilliseconds(milliseconds.count());
        }
    }

    auto sort_by_duration_comparator = [](const std::pair<std::string, Duration>& a, const std::pair<std::string, Duration>& b) {
        return a.second < b.second;
    };

    std::set<std::pair<std::string, Duration>> message_set(status_messages.begin(), status_messages.end());
    std::sort(message_set.begin(), message_set.end(), sort_by_duration_comparator);
}

void VisualizerWrapper::onValueReceived(PlayInfo play_info) {
    most_recent_play_info = play_info;
    updatePlayInfo();
}

void VisualizerWrapper::onValueReceived(AIDrawFunction draw_function) {
    most_recent_ai_draw_function = draw_function;
    draw();
}

void VisualizerWrapper::onValueReceived(WorldDrawFunction draw_function) {
    // TODO: implement
}

void VisualizerWrapper::draw()
{
    // Call the ThunderbotsVisualizer to draw the AI in a threadsafe manner
    // See
    // https://stackoverflow.com/questions/10868946/am-i-forced-to-use-pthread-cond-broadcast-over-pthread-cond-signal-in-order-to/10882705#10882705
    QMetaObject::invokeMethod(visualizer.get(), "draw",
                              Qt::ConnectionType::BlockingQueuedConnection,
                              Q_ARG(WorldDrawFunction, most_recent_world_draw_function),
                              Q_ARG(AIDrawFunction, most_recent_ai_draw_function));
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

void VisualizerWrapper::updatePlayInfo() {
    // Call the ThunderbotsVisualizer to update the Play Info in a threadsafe manner
    // See
    // https://stackoverflow.com/questions/10868946/am-i-forced-to-use-pthread-cond-broadcast-over-pthread-cond-signal-in-order-to/10882705#10882705
    QMetaObject::invokeMethod(visualizer.get(), "updatePlayInfo",
                              Qt::ConnectionType::BlockingQueuedConnection,
                              Q_ARG(PlayInfo, most_recent_play_info));
}
