#pragma once

#include <QtCore/QGenericArgument>
#include <QtWidgets/QApplication>
#include <atomic>
#include <future>
#include <thread>

#include "proto/sensor_msg.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "software/geom/rectangle.h"
#include "software/gui/robot_diagnostics/widgets/robot_diagnostics_gui.h"
#include "software/multithreading/first_in_first_out_threaded_observer.h"
#include "software/multithreading/subject.hpp"
#include "software/multithreading/thread_safe_buffer.hpp"
#include "software/world/world.h"

/**
 * This class wraps our RobotDiagnosticsGUI object which is responsible for allowing users
 * to interact with and debug the robot
 */
class ThreadedRobotDiagnosticsGUI : public FirstInFirstOutThreadedObserver<SensorProto>,
                                    public Subject<TbotsProto::PrimitiveSet>
{
   public:
    ThreadedRobotDiagnosticsGUI() = delete;

    /**
     * Create a new ThreadedRobotDiagnosticsGUI. The argc and argv arguments are required
     * to create a QApplication
     *
     * @param argc The number of arguments being passed
     * @param argv Keyword arguments
     */
    explicit ThreadedRobotDiagnosticsGUI(int argc, char** argv);

    ~ThreadedRobotDiagnosticsGUI() override;

    /**
     * Returns a shared_ptr to a promise that can be waited on, and that will
     * be notified once the ThreadedRobotDiagnosticsGUI has shut down
     *
     * @return a shared_ptr to a promise that can be waited on, and that will
     * be notified once the ThreadedRobotDiagnosticsGUI has been shut down
     */
    std::shared_ptr<std::promise<void>> getTerminationPromise();

   private:
    /**
     * Creates a new RobotDiagnosticsGUI in a new thread and starts running it. These
     * objects must be created in the new thread because the QApplication must be
     * constructed in the thread is will run in, and the RobotDiagnosticsGUI must be
     * created in the same context as the QApplication (which in this case is the new
     * thread).
     *
     * @param argc The number of arguments being passed
     * @param argv Keyword arguments for the RobotDiagnosticsGUI QApplication
     */
    void createAndRunRobotDiagnosticsGUI(int argc, char** argv);

    void onValueReceived(SensorProto sensor_msg) override;

    std::thread run_robot_diagnostics_thread;
    std::thread run_send_primitives_thread;
    std::shared_ptr<std::promise<void>> termination_promise_ptr;

    // Buffers that are shared with the instance of the RobotDiagnosticsGUI so that data
    // can be passed safely
    std::shared_ptr<ThreadSafeBuffer<SensorProto>> sensor_msg_buffer;
    std::shared_ptr<ThreadSafeBuffer<TbotsProto::PrimitiveSet>> primitive_buffer;

    // We don't want to miss any updates so we make the buffer larger
    static constexpr std::size_t sensor_msg_buffer_size = 60;
    static constexpr std::size_t primitive_buffer_size  = 60;
    const unsigned int send_primitive_interval_ms = static_cast<int>(1.0 / 60.0 * 1000);

    std::atomic_bool application_shutting_down;
};
