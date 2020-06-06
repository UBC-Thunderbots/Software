#pragma once

#include <QtCore/QGenericArgument>
#include <QtWidgets/QApplication>
#include <atomic>
#include <future>
#include <thread>

#include "software/gui/robot_diagnostics/widgets/robot_diagnostics.h"
#include "software/multithreading/thread_safe_buffer.h"
#include "software/multithreading/threaded_observer.h"
#include "software/new_geom/rectangle.h"
#include "software/primitive/primitive.h"
#include "software/proto/sensor_msg.pb.h"
#include "software/world/world.h"

/**
 * This class wraps our RobotDiagnostics object which is responsible for allowing users to
 * interact with and debug the robot
 */
class RobotDiagnosticsWrapper : public ThreadedObserver<SensorMsg>
{
   public:
    RobotDiagnosticsWrapper() = delete;

    /**
     * Create a new RobotDiagnostics wrapper. The argc and argv arguments are required
     * to create a QApplication
     *
     * @param argc The number of arguments being passed
     * @param argv Keyword arguments
     */
    explicit RobotDiagnosticsWrapper(int argc, char** argv);

    ~RobotDiagnosticsWrapper() override;

    /**
     * Returns a shared_ptr to a promise that can be waited on, and that will
     * be notified once the RobotDiagnostics has shut down
     *
     * @return a shared_ptr to a promise that can be waited on, and that will
     * be notified once the RobotDiagnostics has been shut down
     */
    std::shared_ptr<std::promise<void>> getTerminationPromise();

   private:
    /**
     * Creates a new RobotDiagnostics in a new thread and starts running it. These
     * objects must be created in the new thread because the QApplication must be
     * constructed in the thread is will run in, and the RobotDiagnostics must be
     * created in the same context as the QApplication (which in this case is the new
     * thread).
     *
     * @param argc The number of arguments being passed
     * @param argv Keyword arguments for the RobotDiagnostics QApplication
     */
    void createAndRunRobotDiagnostics(int argc, char** argv);

    void onValueReceived(SensorMsg sensor_msg) override;

    std::thread run_robot_diagnostics_thread;
    std::shared_ptr<std::promise<void>> termination_promise_ptr;

    // Buffers that are shared with the instance of the RobotDiagnostics so that data can
    // be passed safely
    std::shared_ptr<ThreadSafeBuffer<SensorMsg>> sensor_msg_buffer;
    std::shared_ptr<ThreadSafeBuffer<std::unique_ptr<Primitive>>> primitive_buffer;

    // We don't want to miss any updates so we make the buffer larger
    static constexpr std::size_t sensor_msg_buffer_size = 60;
    static constexpr std::size_t primitive_buffer_size  = 60;

    std::atomic_bool application_shutting_down;
};
