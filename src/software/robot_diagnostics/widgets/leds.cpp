#include "software/robot_diagnostics/widgets/leds.h"

void setupLEDs(Ui::AutoGeneratedMainWidget *widget, std::string &led_mode)
{
    // clang-format off
    widget->comboBox_leds->addItems({
      "Normal Operation",
      "Lamp Test",
      "Wheel 0 Hall Sensors",
      "Wheel 1 Hall Sensors",
      "Wheel 2 Hall Sensors",
      "Wheel 3 Hall Sensors",
      "Dribbler Hall Sensors",
      "Wheel 0 Encoders",
      "Wheel 1 Encoders",
      "Wheel 2 Encoders",
      "Wheel 3 Encoders"});
    // clang-format on

    auto on_leds_changed = [&led_mode](const QString &text) {
        led_mode = text.toStdString();
        // TODO: send some proto
    };

    QWidget::connect(widget->comboBox_leds, &QComboBox::currentTextChanged,
                     on_leds_changed);
}
