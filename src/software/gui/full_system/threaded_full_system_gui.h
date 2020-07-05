#pragma once

#include <QtCore/QGenericArgument>
#include <QtWidgets/QApplication>
#include <atomic>
#include <future>
#include <thread>

#include "software/ai/hl/stp/play_info.h"
#include "software/backend/robot_status.h"
#include "software/gui/drawing/draw_functions.h"
#include "software/gui/full_system/widgets/full_system_gui.h"
#include "software/multithreading/thread_safe_buffer.h"
#include "software/multithreading/threaded_observer.h"
#include "software/new_geom/rectangle.h"
#include "software/proto/sensor_msg.pb.h"
#include "software/world/world.h"

/**
 * This class wraps our FullSystemGUI object which is responsible for
 * visualizing information about our AI, and allowing users to control it.
 */
class ThreadedFullSystemGUI : public ThreadedObserver<World>,
                              public ThreadedObserver<AIDrawFunction>,
                              public ThreadedObserver<PlayInfo>,
                              public ThreadedObserver<SensorMsg>,
                              public ThreadedObserver<RobotStatus>
{
   public:
    explicit ThreadedFullSystemGUI();

    ~ThreadedFullSystemGUI() override;

    void onValueReceived(World world) override;
    void onValueReceived(AIDrawFunction draw_function) override;
    void onValueReceived(PlayInfo play_info) override;
    void onValueReceived(SensorMsg sensor_msg) override;
    void onValueReceived(RobotStatus robot_status) override;

    /**
     * Returns a shared_ptr to a promise that can be waited on, and that will
     * be notified once the FullSystemGUI has shut down
     *
     * @return a shared_ptr to a promise that can be waited on, and that will
     * be notified once the FullSystemGUI has been shut down
     */
    std::shared_ptr<std::promise<void>> getTerminationPromise();

   private:
    /**
     * Creates a new FullSystemGUI in a new thread and starts running it. These
     * objects must be created in the new thread because the QApplication must be
     * constructed in the thread is will run in, and the FullSystemGUI must be
     * created in the same context as the QApplication (which in this case is the new
     * thread).
     */
    void createAndRunFullSystemGUI();

    std::thread run_full_system_gui_thread;
    std::shared_ptr<std::promise<void>> termination_promise_ptr;

    // Buffers that are shared with the instance of the FullSystemGUI so that data can
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
    static constexpr std::size_t WORLD_DRAW_FUNCTIONS_BUFFER_SIZE = 2;
    static constexpr std::size_t AI_DRAW_FUNCTIONS_BUFFER_SIZE    = 2;
    // We only care about the most recent PlayInfo, so the buffer is of size 1
    static constexpr std::size_t PLAY_INFO_BUFFER_SIZE = 1;
    // We don't want to miss any SensorMsg updates so we make the buffer larger
    static constexpr std::size_t SENSOR_MSG_BUFFER_SIZE = 60;
    // We don't want to miss any robot status updates so we make the buffer larger
    static constexpr std::size_t ROBOT_STATUS_BUFFER_SIZE = 60;
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
