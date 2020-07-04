#include "software/gui/standalone_simulator/threaded_standalone_simulator_gui.h"

#include <QtCore/QTimer>
#include <QtWidgets/QApplication>

#include "software/gui/standalone_simulator/widgets/standalone_simulator_gui.h"
#include "software/proto/message_translation/ssl_geometry.h"

ThreadedStandaloneSimulatorGUI::ThreadedStandaloneSimulatorGUI()
    : ThreadedObserver<SSL_WrapperPacket>(),
      termination_promise_ptr(std::make_shared<std::promise<void>>()),
      ssl_wrapper_packet_buffer(std::make_shared<ThreadSafeBuffer<SSL_WrapperPacket>>(
          SSL_WRAPPER_PACKET_BUFFER_SIZE, false)),
      view_area_buffer(
          std::make_shared<ThreadSafeBuffer<Rectangle>>(VIEW_AREA_BUFFER_SIZE, false)),
      application_shutting_down(false),
      remaining_attempts_to_set_view_area(NUM_ATTEMPTS_TO_SET_INITIAL_VIEW_AREA)
{
    run_standalone_simulator_gui_thread = std::thread(
        &ThreadedStandaloneSimulatorGUI::createAndRunStandaloneSimulatorGUI, this);
}

ThreadedStandaloneSimulatorGUI::~ThreadedStandaloneSimulatorGUI()
{
    QCoreApplication* application_ptr = QApplication::instance();
    if (!application_shutting_down.load() && application_ptr != nullptr)
    {
        // Call the Application in a threadsafe manner.
        // https://stackoverflow.com/questions/10868946/am-i-forced-to-use-pthread-cond-broadcast-over-pthread-cond-signal-in-order-to/10882705#10882705
        QMetaObject::invokeMethod(application_ptr, "quit",
                                  Qt::ConnectionType::QueuedConnection);
    }

    run_standalone_simulator_gui_thread.join();
}

void ThreadedStandaloneSimulatorGUI::createAndRunStandaloneSimulatorGUI()
{
    // We mock empty argc and argv since they don't affect the behaviour of the GUI.
    // This way we don't need to pass them all the way down from the start of the
    // program
    char* argv[] = {NULL};
    int argc     = sizeof(argv) / sizeof(char*) - 1;

    // We use raw pointers to have explicit control over the order of destruction.
    // For some reason, putting the QApplication and SimulatorGUI on the stack does
    // not work, despite theoretically having the same order of destruction
    QApplication* application = new QApplication(argc, argv);
    QApplication::connect(application, &QApplication::aboutToQuit,
                          [&]() { application_shutting_down = true; });
    StandaloneSimulatorGUI* standalone_simulator_gui =
        new StandaloneSimulatorGUI(ssl_wrapper_packet_buffer, view_area_buffer);
    standalone_simulator_gui->show();

    // Run the QApplication and all windows / widgets. This function will block
    // until "quit" is called on the QApplication, either by closing all the
    // application windows or calling the destructor of this class
    application->exec();

    // NOTE: The standalone_simulator_gui MUST be deleted before the QApplication. The
    // QApplication manages all the windows, widgets, and event loop so must be destroyed
    // last
    delete standalone_simulator_gui;
    delete application;

    // Let the system know the gui has shut down once the application has
    // stopped running
    termination_promise_ptr->set_value();
}

void ThreadedStandaloneSimulatorGUI::onValueReceived(SSL_WrapperPacket wrapper_packet)
{
    ssl_wrapper_packet_buffer->push(wrapper_packet);

    if (remaining_attempts_to_set_view_area > 0)
    {
        if (wrapper_packet.has_geometry())
        {
            auto field = createField(wrapper_packet.geometry());
            if (field)
            {
                remaining_attempts_to_set_view_area--;
                view_area_buffer->push(field->fieldBoundary());
            }
        }
    }
}

std::shared_ptr<std::promise<void>>
ThreadedStandaloneSimulatorGUI::getTerminationPromise()
{
    return termination_promise_ptr;
}
