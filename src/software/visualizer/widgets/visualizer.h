#pragma once

#include <QtCore/QTimer>
#include <QtWidgets/QMainWindow>

#include "software/ai/hl/stp/play_info.h"
#include "software/backend/robot_status.h"
#include "software/multithreading/thread_safe_buffer.h"
#include "software/visualizer/drawing/draw_functions.h"
#include "software/visualizer/widgets/main_widget.h"
#include "software/util/time/duration.h"

/**
 * This is the main window / application object for the visualizer.
 * Currently it wraps the "main_widget" which is what actually contains
 * all the content.
 *
 * This class uses ThreadSafeBuffers to receive new data, and updates the GUI with
 * the new data at a defined rate using a timer.
 */
class Visualizer : public QMainWindow
{
    Q_OBJECT

   public:
    /**
     * Creates a new Visualizer MainWindow
     *
     * @param world_draw_functions_buffer The buffer used to receive new WorldDrawFunctions
     * @param ai_draw_functions_buffer The buffer used to receive new AIDrawFunctions
     * @param play_info_buffer The buffer used to receive new PlayInfo
     * @param robot_status_buffer The buffer used to receive new RobotStatuses
     */
    explicit Visualizer(
        std::shared_ptr<ThreadSafeBuffer<WorldDrawFunction>> world_draw_functions_buffer,
        std::shared_ptr<ThreadSafeBuffer<AIDrawFunction>> ai_draw_functions_buffer,
        std::shared_ptr<ThreadSafeBuffer<PlayInfo>> play_info_buffer,
        std::shared_ptr<ThreadSafeBuffer<RobotStatus>> robot_status_buffer);

   public slots:
    /**
     * Updates all components of the visualizer, including the World view,
     * PlayInfo, and RobotStatus
     */
    void updateVisualizer();

   private:
    /**
     * Draws all the AI information we want to display in the Visualizer. This includes
     * visualizing the state of the world as well as drawing the AI state we want to show,
     * like planned navigator paths.
     */
    void draw();

    /**
     * Updates and displays newly provided PlayInfo
     */
    void updatePlayInfo();

    /**
     * Updates and displays the newly provided RobotStatus.
     */
    void updateRobotStatus();

    // Because we set the central widget of the MainWindow to this main_widget,
    // the Qt system takes ownership of the pointer and is responsible for
    // de-allocating it so we don't have to
    MainWidget* main_widget;
    QTimer* update_timer;

    std::shared_ptr<ThreadSafeBuffer<WorldDrawFunction>> world_draw_functions_buffer;
    std::shared_ptr<ThreadSafeBuffer<AIDrawFunction>> ai_draw_functions_buffer;
    std::shared_ptr<ThreadSafeBuffer<PlayInfo>> play_info_buffer;
    std::shared_ptr<ThreadSafeBuffer<RobotStatus>> robot_status_buffer;

    WorldDrawFunction most_recent_world_draw_function;
    AIDrawFunction most_recent_ai_draw_function;

    bool initial_view_area_set;

    const Duration update_timer_interval;
};
