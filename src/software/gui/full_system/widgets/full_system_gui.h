#pragma once

#include <QtCore/QRectF>
#include <QtCore/QTimer>
#include <QtWidgets/QMainWindow>

// .ui files are autogenerated to 'ui_<filename>.h`
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/gui/drawing/draw_functions.h"
#include "software/gui/full_system/ui/ui_main_widget.h"
#include "software/multithreading/thread_safe_buffer.h"
#include "software/proto/play_info_msg.pb.h"
#include "software/proto/sensor_msg.pb.h"
#include "software/time/duration.h"

/**
 * This class is the main application window for the FullSystemGUI.
 * It organizes top-level widgets (if needed) into a layout and sets up
 * any callbacks required for asynchronous operations.
 *
 * This class uses ThreadSafeBuffers to receive new data, and consumes this
 * data at a fixed rate by calling functions periodically with a timer.
 */
class FullSystemGUI : public QMainWindow
{
    Q_OBJECT

   public:
    /**
     * Creates a new FullSystemGUI MainWindow
     *
     * @param world_draw_functions_buffer The buffer used to receive new
     * WorldDrawFunctions
     * @param ai_draw_functions_buffer The buffer used to receive new AIDrawFunctions
     * @param play_info_msg_buffer The buffer used to receive new PlayInfoProto
     * @param sensor_msg_buffer The buffer used to receive new SensorProtos
     * @param view_area_buffer The buffer used to receive Rectangles that specify the area
     * of the world to display in the view
     */
    explicit FullSystemGUI(
        std::shared_ptr<ThreadSafeBuffer<WorldDrawFunction>> world_draw_functions_buffer,
        std::shared_ptr<ThreadSafeBuffer<AIDrawFunction>> ai_draw_functions_buffer,
        std::shared_ptr<ThreadSafeBuffer<PlayInfoProto>> play_info_msg_buffer,
        std::shared_ptr<ThreadSafeBuffer<SensorProto>> sensor_msg_buffer,
        std::shared_ptr<ThreadSafeBuffer<Rectangle>> view_area_buffer,
        std::shared_ptr<ThreadSafeBuffer<double>> worlds_received_per_second_buffer,
        std::shared_ptr<ThreadSafeBuffer<double>> primitives_sent_per_second_buffer,
        std::shared_ptr<ThunderbotsConfig> config);

   private:
    /**
     * This function is called periodically by the update_timer to
     * perform various actions at a fixed rate.
     */
    void handleUpdate();

    /**
     * Draws all the AI information we want to display in the FullSystemGUI. This includes
     * visualizing the state of the world as well as drawing the AI state we want to show,
     * like planned navigator paths.
     */
    void draw();

    /**
     * Updates and displays newly provided PlayInfoProto
     */
    void updatePlayInfoProto();

    /**
     * Updates and displays the newly provided SensorProto.
     */
    void updateSensorProto();

    /**
     * Updates the area of the World being drawn in the FullSystemGUI
     */
    void updateDrawViewArea();

    /**
     * Updates the data per second LCDs
     */
    void updateDataPerSecondLCD();

    // The "parent" of each of these widgets is set during construction, meaning that
    // the Qt system takes ownership of the pointer and is responsible for de-allocating
    // it, so we don't have to
    Ui::AutogeneratedFullSystemMainWidget* main_widget;
    QTimer* update_timer;
    QTimer* data_per_second_timer;

    std::shared_ptr<ThreadSafeBuffer<WorldDrawFunction>> world_draw_functions_buffer;
    std::shared_ptr<ThreadSafeBuffer<AIDrawFunction>> ai_draw_functions_buffer;
    std::shared_ptr<ThreadSafeBuffer<PlayInfoProto>> play_info_msg_buffer;
    std::shared_ptr<ThreadSafeBuffer<SensorProto>> sensor_msg_buffer;
    std::shared_ptr<ThreadSafeBuffer<Rectangle>> view_area_buffer;
    std::shared_ptr<ThreadSafeBuffer<double>> worlds_received_per_second_buffer;
    std::shared_ptr<ThreadSafeBuffer<double>> primitives_sent_per_second_buffer;

    WorldDrawFunction most_recent_world_draw_function;
    AIDrawFunction most_recent_ai_draw_function;

    static constexpr double UPDATE_INTERVAL_SECONDS                 = 1.0 / 60.0;
    static constexpr double DATA_PER_SECOND_UPDATE_INTERVAL_SECONDS = 1.0 / 2.0;
};
