#pragma once

#include <QtCore/QGenericArgument>
#include <QtWidgets/QApplication>
#include <atomic>
#include <future>
#include <thread>

#include "software/ai/hl/stp/play_info.h"
#include "software/backend/robot_status.h"
#include "software/multithreading/thread_safe_buffer.h"
#include "software/multithreading/threaded_observer.h"
#include "software/new_geom/rectangle.h"
#include "software/proto/sensor_msg.pb.h"
#include "software/visualizer/drawing/draw_functions.h"
#include "software/visualizer/widgets/visualizer.h"
#include "software/world/world.h"

/**
 * This class wraps our Visualizer object which is responsible for
 * visualizing information about our AI, and allowing users to control it.
 */
class VisualizerWrapper : public ThreadedObserver<World>,
                          public ThreadedObserver<AIDrawFunction>,
                          public ThreadedObserver<PlayInfo>,
                          public ThreadedObserver<SensorMsg>,
                          public ThreadedObserver<RobotStatus>
{
   public:
    VisualizerWrapper() = delete;

    /**
     * Create a new Visualizer wrapper. The argc and argv arguments are required
     * to create a QApplication
     *
     * @param argc The number of arguments being passed
     * @param argv Keyword arguments
     */
    explicit VisualizerWrapper(int argc, char** argv);

    ~VisualizerWrapper() override;

    /**
     * Returns a shared_ptr to a promise that can be waited on, and that will
     * be notified once the Visualizer has shut down
     *
     * @return a shared_ptr to a promise that can be waited on, and that will
     * be notified once the Visualizer has been shut down
     */
    std::shared_ptr<std::promise<void>> getTerminationPromise();

   private:
    /**
     * Creates a new Visualizer in a new thread and starts running it. These
     * objects must be created in the new thread because the QApplication must be
     * constructed in the thread is will run in, and the Visualizer must be
     * created in the same context as the QApplication (which in this case is the new
     * thread).
     *
     * @param argc The number of arguments being passed
     * @param argv Keyword arguments for the Visualizer QApplication
     */
    void createAndRunVisualizer(int argc, char** argv);

    void onValueReceived(World world) override;
    void onValueReceived(AIDrawFunction draw_function) override;
    void onValueReceived(PlayInfo play_info) override;
    void onValueReceived(SensorMsg sensor_msg) override;
    void onValueReceived(RobotStatus robot_status) override;

    std::thread run_visualizer_thread;
    std::shared_ptr<std::promise<void>> termination_promise_ptr;

    // Buffers that are shared with the instance of the Visualizer so that data can
    // be passed safely
    std::shared_ptr<ThreadSafeBuffer<WorldDrawFunction>> world_draw_functions_buffer;
    std::shared_ptr<ThreadSafeBuffer<AIDrawFunction>> ai_draw_functions_buffer;
    std::shared_ptr<ThreadSafeBuffer<PlayInfo>> play_info_buffer;
    std::shared_ptr<ThreadSafeBuffer<SensorMsg>> sensor_msg_buffer;
    std::shared_ptr<ThreadSafeBuffer<RobotStatus>> robot_status_buffer;
    std::shared_ptr<ThreadSafeBuffer<Rectangle>> view_area_buffer;

    // We want to show the most recent world and AI data, but also want things to look
    // smooth if the stream of data isn't perfectly consistent, so we use a very small
    // buffer of 2 values to be responsive while also giving a small buffer for
    // smoothness
    static constexpr std::size_t world_draw_functions_buffer_size = 2;
    static constexpr std::size_t ai_draw_functions_buffer_size    = 2;
    // We only care about the most recent PlayInfo, so the buffer is of size 1
    static constexpr std::size_t play_info_buffer_size = 1;
    // We don't want to miss any SensorMsg updates so we make the buffer larger
    static constexpr std::size_t sensor_msg_buffer_size = 60;
    // We don't want to miss any robot status updates so we make the buffer larger
    static constexpr std::size_t robot_status_buffer_size = 60;
    // We only care about the most recent view area that was requested, so the
    // buffer is of size 1
    static constexpr std::size_t view_area_buffer_size = 1;

    std::atomic_bool application_shutting_down;
    bool initial_view_area_set;
};
