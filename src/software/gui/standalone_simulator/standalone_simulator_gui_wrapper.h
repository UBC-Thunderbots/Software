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
 * This class wraps our SimulatorGUI object which is responsible for
 * visualizing simulation and allowing users to control it.
 */
class StandaloneSimulatorGUIWrapper : public ThreadedObserver<SSL_WrapperPacket>
{
   public:
    StandaloneSimulatorGUIWrapper() = delete;

    /**
     * Create a new Simulator GUI wrapper. The argc and argv arguments are required
     * to create a QApplication
     *
     * @param argc The number of arguments being passed
     * @param argv Keyword arguments
     */
    explicit StandaloneSimulatorGUIWrapper(int argc, char** argv);

    ~StandaloneSimulatorGUIWrapper() override;

    /**
     * Returns a shared_ptr to a promise that can be waited on, and that will
     * be notified once the SimulatorGUI has shut down
     *
     * @return a shared_ptr to a promise that can be waited on, and that will
     * be notified once the SimulatorGUI has been shut down
     */
    std::shared_ptr<std::promise<void>> getTerminationPromise();

    void onValueReceived(SSL_WrapperPacket wrapper_packet) override;

   private:
    /**
     * Creates a new Simulator GUI in a new thread and starts running it. These
     * objects must be created in the new thread because the QApplication must be
     * constructed in the thread is will run in, and the Simulator GUI must be
     * created in the same context as the QApplication (which in this case is the new
     * thread).
     *
     * @param argc The number of arguments being passed
     * @param argv Keyword arguments for the Visualizer QApplication
     */
    void createAndRunStandaloneSimulatorGUI(int argc, char** argv);

    std::thread run_simulator_gui_thread;
    std::shared_ptr<std::promise<void>> termination_promise_ptr;

    // Buffers that are shared with the instance of the Simulator GUI so that data can
    // be passed safely
    std::shared_ptr<ThreadSafeBuffer<SSL_WrapperPacket>> ssl_wrapper_packet_buffer;
    std::shared_ptr<ThreadSafeBuffer<Rectangle>> view_area_buffer;

    // We want to show the most recent data so we use a very small buffer
    static constexpr std::size_t ssl_wrapper_packet_buffer_size = 1;
    // We only care about the most recent view area that was requested, so the
    // buffer is of size 1
    static constexpr std::size_t view_area_buffer_size = 1;
    // When the application starts up we want to set the initial view area
    // to show all the contents nicely. For some reason doing this only
    // once at the start of the program isn't enough, we need to do it several
    // times before it takes effect permanently.
    // 50 is an arbitrary value that is small enough the user doesn't notice
    // they don't have control on startup, but successfully sets the area.
    static constexpr int NUM_ATTEMPTS_TO_SET_INITIAL_VIEW_AREA = 50;

    std::atomic_bool application_shutting_down;
    int remaining_attempts_to_set_view_area;
};
