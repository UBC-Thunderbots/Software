#include "software/gui/robot_diagnostics/widgets/slider.h"

void setupSliderLineEdit(QLineEdit *line_edit, QSlider *slider,
                         std::function<void(double)> value_changed_callback, double min,
                         double max, double slider_step_size)
{
    slider->setMinimum(min * slider_step_size);
    slider->setMaximum(max * slider_step_size);
    slider->setValue(0);
    line_edit->setText(QString::number(0));

    auto on_slider_changed = [line_edit, slider_step_size,
                              value_changed_callback](const int slider_value) {
        double value = slider_value / slider_step_size;
        line_edit->setText(QString::number(value));
        value_changed_callback(value);
    };

    auto on_line_edit_changed = [line_edit, slider, min, max, slider_step_size,
                                 value_changed_callback](const QString &text) {
        bool ok;
        double line_edit_input = text.toDouble(&ok);
        double value;

        // handle corner case of empty input and minus sign
        if ((text.isEmpty()) || (text == "-"))
        {
            slider->setValue(0);
            value = 0;
        }
        else if (ok)
        {
            value = std::clamp(line_edit_input, min, max);
            if (value != line_edit_input)
            {
                line_edit->setText(QString::number(value));
            }
            slider->setValue(value * slider_step_size);
        }
        else
        {
            // line_edit input is invalid so get value from slider
            double value = slider->value() / slider_step_size;
            line_edit->setText(QString::number(value));
        }
        value_changed_callback(value);
    };

    QWidget::connect(slider, &QSlider::valueChanged, on_slider_changed);

    QWidget::connect(line_edit, &QLineEdit::textChanged, on_line_edit_changed);
}
