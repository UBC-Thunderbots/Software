#pragma once

#include <QtCore/QTimer>
#include <QtWidgets/QMainWindow>

#include "software/gui/standalone_simulator/widgets/main_widget.h"
#include "software/multithreading/thread_safe_buffer.h"
#include "software/new_geom/rectangle.h"
#include "software/proto/messages_robocup_ssl_wrapper.pb.h"

/**
 * This is the main window / application object for the StandaloneSimulatorGUI.
 * Currently it wraps the "main_widget" which is what actually contains
 * all the content.
 *
 * This class uses ThreadSafeBuffers to receive new data, and updates the GUI with
 * the new data at a defined rate using a timer.
 */
class StandaloneSimulatorGUI : public QMainWindow
{
    Q_OBJECT

   public:
    /**
     * Creates a new StandaloneSimulatorGUI
     *
     * @param ssl_wrapper_packet_buffer Thye buffer use to receive SSL_WrapperPackets
     * @param view_area_buffer The buffer used to receive Rectangles that specify the area
     * of the world to display in the view
     */
    explicit StandaloneSimulatorGUI(
        std::shared_ptr<ThreadSafeBuffer<SSL_WrapperPacket>> ssl_wrapper_packet_buffer,
        std::shared_ptr<ThreadSafeBuffer<Rectangle>> view_area_buffer);

   public slots:
    /**
     * Draws all the contents of the most recently received SSL_WrapperPacket,
     * and updates the view area if necessary.
     */
    void drawSimulatorContents();

   private:
    /**
     * Updates the area of the World being drawn in the Visualizer
     */
    void updateDrawViewArea();

    // Because we set the central widget of the MainWindow to this main_widget,
    // the Qt system takes ownership of the pointer and is responsible for
    // de-allocating it so we don't have to
    MainWidget* main_widget;
    QTimer* update_timer;

    std::shared_ptr<ThreadSafeBuffer<SSL_WrapperPacket>> ssl_wrapper_packet_buffer;
    std::shared_ptr<ThreadSafeBuffer<Rectangle>> view_area_buffer;

    static constexpr double DRAW_UPDATE_INTERVAL_SECONDS = 1.0 / 60.0;
};
