#include "software/gui/full_system/threaded_full_system_gui.h"

#include <QtCore/QTimer>
#include <QtWidgets/QApplication>

#include "software/gui/drawing/world.h"
#include "software/parameter/dynamic_parameters.h"

ThreadedFullSystemGUI::ThreadedFullSystemGUI()
    : ThreadedObserver<World>(),
      ThreadedObserver<AIDrawFunction>(),
      ThreadedObserver<PlayInfo>(),
      ThreadedObserver<SensorMsg>(),
      ThreadedObserver<RobotStatus>(),
      termination_promise_ptr(std::make_shared<std::promise<void>>()),
      world_draw_functions_buffer(std::make_shared<ThreadSafeBuffer<WorldDrawFunction>>(
          WORLD_DRAW_FUNCTIONS_BUFFER_SIZE, false)),
      ai_draw_functions_buffer(std::make_shared<ThreadSafeBuffer<AIDrawFunction>>(
          AI_DRAW_FUNCTIONS_BUFFER_SIZE, false)),
      play_info_buffer(
          std::make_shared<ThreadSafeBuffer<PlayInfo>>(PLAY_INFO_BUFFER_SIZE, false)),
      sensor_msg_buffer(
          std::make_shared<ThreadSafeBuffer<SensorMsg>>(SENSOR_MSG_BUFFER_SIZE)),
      robot_status_buffer(
          std::make_shared<ThreadSafeBuffer<RobotStatus>>(ROBOT_STATUS_BUFFER_SIZE)),
      view_area_buffer(
          std::make_shared<ThreadSafeBuffer<Rectangle>>(VIEW_AREA_BUFFER_SIZE, false)),
      application_shutting_down(false),
      remaining_attempts_to_set_view_area(NUM_ATTEMPTS_TO_SET_INITIAL_VIEW_AREA)
{
    run_full_system_gui_thread =
        std::thread(&ThreadedFullSystemGUI::createAndRunFullSystemGUI, this);
}

ThreadedFullSystemGUI::~ThreadedFullSystemGUI()
{
    QCoreApplication* application_ptr = QApplication::instance();
    if (!application_shutting_down.load() && application_ptr != nullptr)
    {
        // Call the Application in a threadsafe manner.
        // https://stackoverflow.com/questions/10868946/am-i-forced-to-use-pthread-cond-broadcast-over-pthread-cond-signal-in-order-to/10882705#10882705
        QMetaObject::invokeMethod(application_ptr, "quit",
                                  Qt::ConnectionType::QueuedConnection);
    }

    run_full_system_gui_thread.join();
}

void ThreadedFullSystemGUI::createAndRunFullSystemGUI()
{
    // We mock empty argc and argv since they don't affect the behaviour of the GUI.
    // This way we don't need to pass them all the way down from the start of the
    // program
    char* argv[] = {NULL};
    int argc     = sizeof(argv) / sizeof(char*) - 1;

    // We use raw pointers to have explicit control over the order of destruction.
    // For some reason, putting the QApplication and FullSystemGUI on the stack does
    // not work, despite theoretically having the same order of destruction
    QApplication* application = new QApplication(argc, argv);
    QApplication::connect(application, &QApplication::aboutToQuit,
                          [&]() { application_shutting_down = true; });
    FullSystemGUI* full_system_gui =
        new FullSystemGUI(world_draw_functions_buffer, ai_draw_functions_buffer,
                          play_info_buffer, sensor_msg_buffer, robot_status_buffer,
                          view_area_buffer, MutableDynamicParameters);
    full_system_gui->show();

    // Run the QApplication and all windows / widgets. This function will block
    // until "quit" is called on the QApplication, either by closing all the
    // application windows or calling the destructor of this class
    application->exec();

    // NOTE: The full_system MUST be deleted before the QApplication. The QApplication
    // manages all the windows, widgets, and event loop so must be destroyed last
    delete full_system_gui;
    delete application;

    // Let the system know the full_system has shut down once the application has
    // stopped running
    termination_promise_ptr->set_value();
}

void ThreadedFullSystemGUI::onValueReceived(World world)
{
    auto world_draw_function = getDrawWorldFunction(world);
    world_draw_functions_buffer->push(world_draw_function);

    if (remaining_attempts_to_set_view_area > 0)
    {
        remaining_attempts_to_set_view_area--;
        view_area_buffer->push(world.field().fieldBoundary());
    }
}

void ThreadedFullSystemGUI::onValueReceived(AIDrawFunction draw_function)
{
    ai_draw_functions_buffer->push(draw_function);
}

void ThreadedFullSystemGUI::onValueReceived(PlayInfo play_info)
{
    play_info_buffer->push(play_info);
}

void ThreadedFullSystemGUI::onValueReceived(SensorMsg sensor_msg)
{
    sensor_msg_buffer->push(sensor_msg);
}

void ThreadedFullSystemGUI::onValueReceived(RobotStatus robot_status)
{
    robot_status_buffer->push(robot_status);
}

std::shared_ptr<std::promise<void>> ThreadedFullSystemGUI::getTerminationPromise()
{
    return termination_promise_ptr;
}
