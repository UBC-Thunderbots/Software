#include "software/gui/robot_diagnostics/robot_diagnostics_wrapper.h"

#include <QtCore/QTimer>
#include <QtWidgets/QApplication>

RobotDiagnosticsWrapper::RobotDiagnosticsWrapper(int argc, char** argv)
    : ThreadedObserver<SensorMsg>(),
      termination_promise_ptr(std::make_shared<std::promise<void>>()),
      sensor_msg_buffer(
          std::make_shared<ThreadSafeBuffer<SensorMsg>>(sensor_msg_buffer_size)),
      primitive_buffer(std::make_shared<ThreadSafeBuffer<std::unique_ptr<Primitive>>>(
          primitive_buffer_size)),
      application_shutting_down(false)
{
    run_robot_diagnostics_thread = std::thread(
        &RobotDiagnosticsWrapper::createAndRunRobotDiagnostics, this, argc, argv);
}

RobotDiagnosticsWrapper::~RobotDiagnosticsWrapper()
{
    QCoreApplication* application_ptr = QApplication::instance();
    if (!application_shutting_down.load() && application_ptr != nullptr)
    {
        // Call the Application in a threadsafe manner.
        // https://stackoverflow.com/questions/10868946/am-i-forced-to-use-pthread-cond-broadcast-over-pthread-cond-signal-in-order-to/10882705#10882705
        QMetaObject::invokeMethod(application_ptr, "quit",
                                  Qt::ConnectionType::QueuedConnection);
    }

    run_robot_diagnostics_thread.join();
}

void RobotDiagnosticsWrapper::createAndRunRobotDiagnostics(int argc, char** argv)
{
    // We use raw pointers to have explicit control over the order of destruction.
    // For some reason, putting the QApplication and RobotDiagnostics on the stack does
    // not work, despite theoretically having the same order of destruction
    QApplication* application = new QApplication(argc, argv);
    QApplication::connect(application, &QApplication::aboutToQuit,
                          [&]() { application_shutting_down = true; });
    RobotDiagnostics* robot_diagnostics =
        new RobotDiagnostics(sensor_msg_buffer, primitive_buffer);
    robot_diagnostics->show();

    // Run the QApplication and all windows / widgets. This function will block
    // until "quit" is called on the QApplication, either by closing all the
    // application windows or calling the destructor of this class
    application->exec();

    // NOTE: The robot_diagnostics MUST be deleted before the QApplication. The
    // QApplication manages all the windows, widgets, and event loop so must be destroyed
    // last
    delete robot_diagnostics;
    delete application;

    // Let the system know the robot_diagnostics has shut down once the application has
    // stopped running
    termination_promise_ptr->set_value();
}

void RobotDiagnosticsWrapper::onValueReceived(SensorMsg sensor_msg)
{
    sensor_msg_buffer->push(sensor_msg);
}

std::shared_ptr<std::promise<void>> RobotDiagnosticsWrapper::getTerminationPromise()
{
    return termination_promise_ptr;
}
