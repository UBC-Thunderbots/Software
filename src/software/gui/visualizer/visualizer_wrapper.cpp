#include "software/gui/visualizer/visualizer_wrapper.h"

#include <QtCore/QTimer>
#include <QtWidgets/QApplication>

#include "software/gui/drawing/world.h"
#include "software/parameter/dynamic_parameters.h"

VisualizerWrapper::VisualizerWrapper(int argc, char** argv)
    : ThreadedObserver<World>(),
      ThreadedObserver<AIDrawFunction>(),
      ThreadedObserver<PlayInfo>(),
      ThreadedObserver<SensorMsg>(),
      ThreadedObserver<RobotStatus>(),
      termination_promise_ptr(std::make_shared<std::promise<void>>()),
      world_draw_functions_buffer(std::make_shared<ThreadSafeBuffer<WorldDrawFunction>>(
          world_draw_functions_buffer_size, false)),
      ai_draw_functions_buffer(std::make_shared<ThreadSafeBuffer<AIDrawFunction>>(
          ai_draw_functions_buffer_size, false)),
      play_info_buffer(
          std::make_shared<ThreadSafeBuffer<PlayInfo>>(play_info_buffer_size, false)),
      sensor_msg_buffer(
          std::make_shared<ThreadSafeBuffer<SensorMsg>>(sensor_msg_buffer_size)),
      robot_status_buffer(
          std::make_shared<ThreadSafeBuffer<RobotStatus>>(robot_status_buffer_size)),
      view_area_buffer(
          std::make_shared<ThreadSafeBuffer<Rectangle>>(view_area_buffer_size, false)),
      application_shutting_down(false),
      initial_view_area_set(false)
{
    run_visualizer_thread =
        std::thread(&VisualizerWrapper::createAndRunVisualizer, this, argc, argv);
}

VisualizerWrapper::~VisualizerWrapper()
{
    QCoreApplication* application_ptr = QApplication::instance();
    if (!application_shutting_down.load() && application_ptr != nullptr)
    {
        // Call the Application in a threadsafe manner.
        // https://stackoverflow.com/questions/10868946/am-i-forced-to-use-pthread-cond-broadcast-over-pthread-cond-signal-in-order-to/10882705#10882705
        QMetaObject::invokeMethod(application_ptr, "quit",
                                  Qt::ConnectionType::QueuedConnection);
    }

    run_visualizer_thread.join();
}

void VisualizerWrapper::createAndRunVisualizer(int argc, char** argv)
{
    // We use raw pointers to have explicit control over the order of destruction.
    // For some reason, putting the QApplication and Visualizer on the stack does
    // not work, despite theoretically having the same order of destruction
    QApplication* application = new QApplication(argc, argv);
    QApplication::connect(application, &QApplication::aboutToQuit,
                          [&]() { application_shutting_down = true; });
    Visualizer* visualizer =
        new Visualizer(world_draw_functions_buffer, ai_draw_functions_buffer,
                       play_info_buffer, sensor_msg_buffer, robot_status_buffer,
                       view_area_buffer, MutableDynamicParameters);
    visualizer->show();

    // Run the QApplication and all windows / widgets. This function will block
    // until "quit" is called on the QApplication, either by closing all the
    // application windows or calling the destructor of this class
    application->exec();

    // NOTE: The visualizer MUST be deleted before the QApplication. The QApplication
    // manages all the windows, widgets, and event loop so must be destroyed last
    delete visualizer;
    delete application;

    // Let the system know the visualizer has shut down once the application has
    // stopped running
    termination_promise_ptr->set_value();
}

void VisualizerWrapper::onValueReceived(World world)
{
    auto world_draw_function = getDrawWorldFunction(world);
    world_draw_functions_buffer->push(world_draw_function);

    if (!initial_view_area_set)
    {
        initial_view_area_set = true;
        view_area_buffer->push(world.field().fieldBoundary());
    }
}

void VisualizerWrapper::onValueReceived(AIDrawFunction draw_function)
{
    ai_draw_functions_buffer->push(draw_function);
}

void VisualizerWrapper::onValueReceived(PlayInfo play_info)
{
    play_info_buffer->push(play_info);
}

void VisualizerWrapper::onValueReceived(SensorMsg sensor_msg)
{
    sensor_msg_buffer->push(sensor_msg);
}

void VisualizerWrapper::onValueReceived(RobotStatus robot_status)
{
    robot_status_buffer->push(robot_status);
}

std::shared_ptr<std::promise<void>> VisualizerWrapper::getTerminationPromise()
{
    return termination_promise_ptr;
}
