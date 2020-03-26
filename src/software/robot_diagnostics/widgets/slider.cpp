#include "software/robot_diagnostics/widgets/slider.h"

void setupSliderLineEdit(QLineEdit *line_edit, QSlider *slider, double &value, double min,
                         double max, double scaling)
{
    slider->setMinimum(min * scaling);
    slider->setMaximum(max * scaling);
    slider->setValue(0);
    line_edit->setText(QString::number(0));

    auto on_slider_changed = [&value, line_edit, scaling](const int slider_value) {
        value = slider_value / scaling;
        line_edit->setText(QString::number(value));
    };

    auto on_line_edit_changed = [&value, line_edit, slider, min, max,
                                 scaling](const QString &text) {
        bool ok;
        double line_edit_input = text.toDouble(&ok);

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
            slider->setValue(value * scaling);
        }
        else
        {
            line_edit->setText(QString::number(value));
        }
    };

    QWidget::connect(slider, &QSlider::valueChanged, on_slider_changed);

    QWidget::connect(line_edit, &QLineEdit::textChanged, on_line_edit_changed);
}
