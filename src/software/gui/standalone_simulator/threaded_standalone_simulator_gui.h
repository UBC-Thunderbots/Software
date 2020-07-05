#pragma once

#include <QtCore/QGenericArgument>
#include <QtWidgets/QApplication>
#include <atomic>
#include <future>
#include <thread>

#include "software/multithreading/thread_safe_buffer.h"
#include "software/multithreading/threaded_observer.h"
#include "software/new_geom/rectangle.h"
#include "software/proto/messages_robocup_ssl_wrapper.pb.h"

/**
 * This class wraps a StandaloneSimulatorGUI object so it can run independently
 * in its own thread. This way the GUI can be rendered and handle events separately
 * without affecting the main application.
 */
class ThreadedStandaloneSimulatorGUI : public ThreadedObserver<SSL_WrapperPacket>
{
   public:
    explicit ThreadedStandaloneSimulatorGUI();

    ~ThreadedStandaloneSimulatorGUI() override;

    /**
     * Returns a shared_ptr to a promise that can be waited on, and that will
     * be notified once the GUI has shut down
     *
     * @return a shared_ptr to a promise that can be waited on, and that will
     * be notified once the GUI has been shut down
     */
    std::shared_ptr<std::promise<void>> getTerminationPromise();

    void onValueReceived(SSL_WrapperPacket wrapper_packet) override;

   private:
    /**
     * Creates a StandaloneSimulatorGUI in a new thread and starts running it. These
     * objects must be created in the new thread because the QApplication must be
     * constructed in the thread it will run in, and the StandaloneSimulatorGUI must be
     * created in the same context as the QApplication (which in this case is the new
     * thread).
     */
    void createAndRunStandaloneSimulatorGUI();

    std::thread run_standalone_simulator_gui_thread;
    std::shared_ptr<std::promise<void>> termination_promise_ptr;

    // Buffers that are shared with the instance of the GUI so that data can
    // be passed safely
    std::shared_ptr<ThreadSafeBuffer<SSL_WrapperPacket>> ssl_wrapper_packet_buffer;
    std::shared_ptr<ThreadSafeBuffer<Rectangle>> view_area_buffer;

    // We want to show the most recent data so we use a very small buffer
    static constexpr std::size_t SSL_WRAPPER_PACKET_BUFFER_SIZE = 1;
    // We only care about the most recent view area that was requested, so the
    // buffer is of size 1
    static constexpr std::size_t VIEW_AREA_BUFFER_SIZE = 1;
    // When the application starts up we want to set the initial view area
    // to show all the contents nicely. For some reason doing this only
    // once at the start of the program isn't enough, the GUI seems to need
    // a few iterations to fully render everything before the view area will
    // be set correctly (otherwise the contents tend to be zoomed-out and
    // offset). This is set to an arbitrary value that is small enough the
    // user doesn't notice they don't have control on startup, but
    // successfully sets the area.
    static constexpr int NUM_ATTEMPTS_TO_SET_INITIAL_VIEW_AREA = 50;

    std::atomic_bool application_shutting_down;
    int remaining_attempts_to_set_view_area;
};
