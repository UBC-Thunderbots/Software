#include "software/gui/robot_diagnostics/widgets/robot_diagnostics_gui.h"

RobotDiagnosticsGUI::RobotDiagnosticsGUI(
    std::shared_ptr<ThreadSafeBuffer<SensorProto>> sensor_msg_buffer,
    std::shared_ptr<ThreadSafeBuffer<TbotsProto::PrimitiveSet>> primitive_buffer,
    QWidget* parent)
    : QMainWindow(parent),
      chip_pressed(false),
      kick_pressed(false),
      main_widget(new Ui::AutoGeneratedMainWidget()),
      primitive_buffer(primitive_buffer),
      sensor_msg_buffer(sensor_msg_buffer),
      update_timer(new QTimer(this)),
      push_primitive_timer(new QTimer(this)),
      update_timer_interval(Duration::fromSeconds(1.0 / 60.0)),
      push_primitive_timer_interval(Duration::fromSeconds(1.0 / 30.0))
{
    // Handles all the setup of the generated UI components and adds the components
    // to this widget
    main_widget->setupUi(this);

    // StrongFocus means that the widget will more aggressively capture focus when
    // clicked. Specifically, we do this so that when the user clicks outside of the
    // QLineEdits used for Parameters, the QLineEdit will lose focus.
    // https://www.qtcentre.org/threads/41128-Need-to-implement-in-place-line-edit-unable-to-get-lose-focus-of-QLineEdit
    setFocusPolicy(Qt::StrongFocus);
    connect(update_timer, &QTimer::timeout, this,
            &RobotDiagnosticsGUI::updateRobotDiagnostics);
    connect(push_primitive_timer, &QTimer::timeout, this, [this]() {
        auto direct_control_primitive = createDirectControlPrimitiveFromUI();
        pushPrimitiveSetToBuffer(*direct_control_primitive);
    });
    update_timer->start(static_cast<int>(update_timer_interval.getMilliseconds()));
    push_primitive_timer->start(
        static_cast<int>(push_primitive_timer_interval.getMilliseconds()));

    setupWidgets();
}

void RobotDiagnosticsGUI::onChickerStateChanged(double chicker_power,
                                                ChargeMode charge_mode,
                                                ChickMode chick_mode)
{
    if (chick_mode == ChickMode::CHIP)
    {
        chip_pressed = true;
    }
    else if (chick_mode == ChickMode::KICK)
    {
        kick_pressed = true;
    }
    auto direct_control_primitive = createDirectControlPrimitiveFromUI();
    pushPrimitiveSetToBuffer(*direct_control_primitive);
}

void RobotDiagnosticsGUI::onDirectVelocityPowerChanged(
    double direct_per_wheel_power, DirectVelocityMode direct_velocity_mode)
{
    auto direct_control_primitive = createDirectControlPrimitiveFromUI();
    pushPrimitiveSetToBuffer(*direct_control_primitive);
}

void RobotDiagnosticsGUI::onDirectPerWheelPowerChanged(
    double direct_per_wheel_power, DirectPerWheelMode direct_per_wheel_mode)
{
    auto direct_control_primitive = createDirectControlPrimitiveFromUI();
    pushPrimitiveSetToBuffer(*direct_control_primitive);
}

void RobotDiagnosticsGUI::onDribblerPowerChanged(double dribbler_power)
{
    auto direct_control_primitive = createDirectControlPrimitiveFromUI();
    pushPrimitiveSetToBuffer(*direct_control_primitive);
}

std::unique_ptr<TbotsProto::Primitive>
RobotDiagnosticsGUI::createDirectControlPrimitiveFromUI()
{
    auto direct_control_primitive_msg = std::make_unique<TbotsProto::Primitive>();

    setWheelControlPrimitiveFromUI(*direct_control_primitive_msg);
    setChargeModeFromUI(*direct_control_primitive_msg);
    setChickCommandPrimitiveFromUI(*direct_control_primitive_msg);

    direct_control_primitive_msg->mutable_direct_control()->set_dribbler_speed_rpm(
        main_widget->lineEdit_dribbler_power->text().toFloat());

    return direct_control_primitive_msg;
}

void RobotDiagnosticsGUI::pushPrimitiveSetToBuffer(
    TbotsProto::Primitive& primitive_msg)
{
    auto primitive_set_msg = std::make_unique<TbotsProto::PrimitiveSet>();
    *(primitive_set_msg->mutable_time_sent()) = *createCurrentTimestamp();

    auto& robot_primitives_map = *primitive_set_msg->mutable_robot_primitives();
    robot_primitives_map[robot_selection] = primitive_msg;

    primitive_buffer->push(*primitive_set_msg);
}

void RobotDiagnosticsGUI::setChargeModeFromUI(
        TbotsProto::Primitive& primitive_msg)
{
    if (main_widget->buttonGroup_charge_state->checkedButton() ==
        main_widget->radioButton_charge)
    {
        primitive_msg.mutable_direct_control()->set_charge_mode(
            TbotsProto::DirectControlPrimitive_ChargeMode_CHARGE);
    }
    else if (main_widget->buttonGroup_charge_state->checkedButton() ==
             main_widget->radioButton_discharge)
    {
        primitive_msg.mutable_direct_control()->set_charge_mode(
            TbotsProto::DirectControlPrimitive_ChargeMode_DISCHARGE);
    }
    else if (main_widget->buttonGroup_charge_state->checkedButton() ==
             main_widget->radioButton_float)
    {
        primitive_msg.mutable_direct_control()->set_charge_mode(
            TbotsProto::DirectControlPrimitive_ChargeMode_FLOAT);
    }
}

void RobotDiagnosticsGUI::setWheelControlPrimitiveFromUI(
        TbotsProto::Primitive& primitive_msg)
{
    // Uses currently selected tab widget to decide which wheel control primitive to make
    switch (main_widget->tabWidget->currentIndex())
    {
        case static_cast<int>(WheelControlTab::DIRECT_PER_WHEEL):
            primitive_msg.mutable_direct_control()
                ->mutable_direct_per_wheel_control()
                ->set_front_left_wheel_rpm(
                    main_widget->lineEdit_direct_per_wheel_fl->text().toFloat());
            primitive_msg.mutable_direct_control()
                ->mutable_direct_per_wheel_control()
                ->set_front_right_wheel_rpm(
                    main_widget->lineEdit_direct_per_wheel_fr->text().toFloat());
            primitive_msg.mutable_direct_control()
                ->mutable_direct_per_wheel_control()
                ->set_back_left_wheel_rpm(
                    main_widget->lineEdit_direct_per_wheel_bl->text().toFloat());
            primitive_msg.mutable_direct_control()
                ->mutable_direct_per_wheel_control()
                ->set_back_right_wheel_rpm(
                    main_widget->lineEdit_direct_per_wheel_br->text().toFloat());
            break;
        case static_cast<int>(WheelControlTab::DIRECT_VELOCITY):
            primitive_msg.mutable_direct_control()
                ->mutable_direct_velocity_control()
                ->mutable_velocity()
                ->set_x_component_meters(
                    main_widget->lineEdit_direct_velocity_x->text().toFloat());
            primitive_msg.mutable_direct_control()
                ->mutable_direct_velocity_control()
                ->mutable_velocity()
                ->set_y_component_meters(
                    main_widget->lineEdit_direct_velocity_y->text().toFloat());

            auto angular_velocity_msg =
                createAngularVelocityProto(AngularVelocity::fromDegrees(
                    main_widget->lineEdit_direct_velocity_theta->text().toDouble()));
            *(primitive_msg.mutable_direct_control()
                  ->mutable_direct_velocity_control()
                  ->mutable_angular_velocity()) = *angular_velocity_msg;
            break;
    }
}

void RobotDiagnosticsGUI::setChickCommandPrimitiveFromUI(
        TbotsProto::Primitive& primitive_msg)
{
    // Uses the auto chick button group to decide which chick command primitive to make
    if (main_widget->buttonGroup_autochick->checkedButton() ==
        main_widget->radioButton_autochick_none)
    {
        // Checks if chip/kick radio buttons were pressed
        if (chip_pressed)
        {
            primitive_msg.mutable_direct_control()->set_chip_distance_meters(
                main_widget->lineEdit_chicker_power->text().toFloat());
            chip_pressed = false;
        }
        else if (kick_pressed)
        {
            primitive_msg.mutable_direct_control()->set_kick_speed_meters_per_second(
                main_widget->lineEdit_chicker_power->text().toFloat());
            kick_pressed = false;
        }
    }
    else if (main_widget->buttonGroup_autochick->checkedButton() ==
             main_widget->radioButton_autochip)
    {
        primitive_msg.mutable_direct_control()->set_autochip_distance_meters(
            main_widget->lineEdit_chicker_power->text().toFloat());
    }
    else if (main_widget->buttonGroup_autochick->checkedButton() ==
             main_widget->radioButton_autokick)
    {
        primitive_msg.mutable_direct_control()->set_autokick_speed_meters_per_second(
            main_widget->lineEdit_chicker_power->text().toFloat());
    }
}

void RobotDiagnosticsGUI::setupWidgets()
{
    setupChicker(main_widget, boost::bind(&RobotDiagnosticsGUI::onChickerStateChanged,
                                          this, _1, _2, _3));
    setupDribbler(main_widget,
                  boost::bind(&RobotDiagnosticsGUI::onDribblerPowerChanged, this, _1));
    setupDrive(
        main_widget,
        boost::bind(&RobotDiagnosticsGUI::onDirectPerWheelPowerChanged, this, _1, _2),
        boost::bind(&RobotDiagnosticsGUI::onDirectVelocityPowerChanged, this, _1, _2));
    setupSensorStatus(main_widget);
    setupLEDs(main_widget, led_mode);
    setupRobotSelection(main_widget, robot_selection);
    setupRobotStatusTable(main_widget->robot_status_table_widget);
}

void RobotDiagnosticsGUI::updateRobotDiagnostics()
{
    std::optional<SensorProto> sensor_msg =
        sensor_msg_buffer->popLeastRecentlyAddedValue();
    while (sensor_msg)
    {
        updateSensorStatus(main_widget, sensor_msg.value());

        // update robot status table
        for (const auto& robot_msg : sensor_msg.value().robot_status_msgs())
        {
            main_widget->robot_status_table_widget->updateRobotStatus(robot_msg);
        }

        sensor_msg = sensor_msg_buffer->popLeastRecentlyAddedValue();
    }
}
