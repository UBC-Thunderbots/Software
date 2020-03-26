#include "software/robot_diagnostics/widgets/dribbler.h"

void setupDribbler(Ui::AutoGeneratedMainWidget *widget, double &dribbler_power)
{
    setupSliderLineEdit(widget->lineEdit_dribbler_power, widget->slider_dribbler_power,
                        dribbler_power, 0.0, 99.0, 1.0);

    auto dribbler_stop_pushed = [widget]() {
        widget->slider_dribbler_power->setValue(0);
        widget->lineEdit_dribbler_power->setText(QString::number(0));

        // TODO (Issue #1229): send some proto
    };

    QWidget::connect(widget->pushButton_dribbler_stop, &QPushButton::clicked,
                     dribbler_stop_pushed);
}
