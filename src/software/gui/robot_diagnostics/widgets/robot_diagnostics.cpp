#include "software/gui/robot_diagnostics/widgets/robot_diagnostics.h"

#include <iostream>

#include "software/gui/generic_widgets/robot_status/robot_status.h"

RobotDiagnostics::RobotDiagnostics(
    std::shared_ptr<ThreadSafeBuffer<SensorMsg>> sensor_msg_buffer,
    std::shared_ptr<ThreadSafeBuffer<std::unique_ptr<Primitive>>> primitive_buffer,
    QWidget* parent)
    : sensor_msg_buffer(sensor_msg_buffer),
      primitive_buffer(primitive_buffer),
      QMainWindow(parent),
      main_widget(new Ui::AutoGeneratedMainWidget())
{
    // Handles all the setup of the generated UI components and adds the components
    // to this widget
    main_widget->setupUi(this);

    // StrongFocus means that the widget will more aggressively capture focus when
    // clicked. Specifically, we do this so that when the user clicks outside of the
    // QLineEdits used for Parameters, the QLineEdit will lose focus.
    // https://www.qtcentre.org/threads/41128-Need-to-implement-in-place-line-edit-unable-to-get-lose-focus-of-QLineEdit
    setFocusPolicy(Qt::StrongFocus);

    dribbler_power_changed_callback = [](double dribbler_power) {
        // TODO (Issue #1229): push some proto
    };

    direct_per_wheel_power_changed_callback =
        [](double direct_per_wheel_power, DirectPerWheelMode direct_per_wheel_mode) {
            // TODO (Issue #1229): push some proto
        };

    direct_velocity_power_changed_callback = [](double direct_per_wheel_power,
                                                DirectVelocityMode direct_velocity_mode) {
        // TODO (Issue #1229): push some proto
    };

    chicker_state_changed_callback = [](double chicker_power, ChargeMode charge_mode,
                                        ChickMode chick_mode) {
        // TODO (Issue #1229): push some proto
    };


    setupChicker(main_widget, chicker_state_changed_callback);
    setupDribbler(main_widget, dribbler_power_changed_callback);
    setupDrive(main_widget, direct_per_wheel_power_changed_callback,
               direct_velocity_power_changed_callback);
    setupFeedback(main_widget);
    setupLEDs(main_widget, led_mode);
    setupRobotSelection(main_widget, robot_selection);
    setupRobotStatusTable(main_widget->robot_status_table_widget);

    update();
}
