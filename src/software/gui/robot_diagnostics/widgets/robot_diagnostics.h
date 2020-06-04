#pragma once

#include <QtWidgets/QMainWindow>
#include <memory>

#include "software/gui/robot_diagnostics/ui/ui_main_widget.h"
#include "software/gui/robot_diagnostics/widgets/chicker.h"
#include "software/gui/robot_diagnostics/widgets/dribbler.h"
#include "software/gui/robot_diagnostics/widgets/drive.h"
#include "software/gui/robot_diagnostics/widgets/feedback.h"
#include "software/gui/robot_diagnostics/widgets/leds.h"
#include "software/gui/robot_diagnostics/widgets/robot_selection.h"
#include "software/multithreading/thread_safe_buffer.h"
#include "software/primitive/primitive.h"
#include "software/proto/sensor_msg.pb.h"

// Forward declare the name of the top-level GUI class defined in main_widget.ui
namespace Ui
{
    class AutoGeneratedMainWidget;
}

/**
 * This is the main window for the robot diagnostics.
 *
 * This class uses ThreadSafeBuffers to send/receive new data, and updates the GUI with
 * the new data at a defined rate using a timer.
 */
class RobotDiagnostics : public QMainWindow
{
    Q_OBJECT

   public:
    /**
     * Creates a new Visualizer MainWindow
     *
     * @param sensor_msg_buffer The buffer used to receive new SensorMsgs
     * @param primitive_buffer The buffer used to send new Primitives
     * @param parent parent widget
     */
    explicit RobotDiagnostics(
        std::shared_ptr<ThreadSafeBuffer<SensorMsg>> sensor_msg_buffer,
        std::shared_ptr<ThreadSafeBuffer<std::unique_ptr<Primitive>>> primitive_buffer,
        QWidget* parent = nullptr);

   private:
    // The "parent" of each of these widgets is set during construction; meaning that
    // the Qt system takes ownership of the pointer and is responsible for de-allocating
    // it; so we don't have to
    Ui::AutoGeneratedMainWidget* main_widget;

    // Indicates which robot we're communicating with
    unsigned int robot_selection;

    // dribbler chicker controls
    double dribbler_power;
    double chicker_power;
    bool chicker_autochick;
    ChargeMode chicker_charge_mode;
    ChickMode chicker_chick_mode;

    // Callbacks
    std::function<void(double, ChargeMode, ChickMode)> chicker_state_changed_callback;
    std::function<void(double)> dribbler_power_changed_callback;
    std::function<void(double, DirectPerWheelMode)>
        direct_per_wheel_power_changed_callback;
    std::function<void(double, DirectVelocityMode)>
        direct_velocity_power_changed_callback;

    double direct_per_wheel_power_fl;
    double direct_per_wheel_power_fr;
    double direct_per_wheel_power_bl;
    double direct_per_wheel_power_br;
    double matrix_x_vel;
    double matrix_y_vel;
    double matrix_angular_vel;
    std::string led_mode;
    std::shared_ptr<ThreadSafeBuffer<SensorMsg>> sensor_msg_buffer;
    std::shared_ptr<ThreadSafeBuffer<std::unique_ptr<Primitive>>> primitive_buffer;
};

void setupFeedbackDisplay(Ui::AutoGeneratedMainWidget* widget);

void setupDribblerTemperature(Ui::AutoGeneratedMainWidget* widget);
