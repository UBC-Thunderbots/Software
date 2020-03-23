#include "software/robot_diagnostics/widgets/slider.h"

void setupSliderLineEdit(QLineEdit *line_edit, QSlider *slider, int &value, int min,
                         int max)
{
    slider->setMinimum(min);
    slider->setMaximum(max);

    auto on_slider_changed = [&value, line_edit](const int slider_value) {
        line_edit->setText(QString::number(slider_value));
        value = slider_value;
    };

    auto on_line_edit_changed = [&value, line_edit, slider, min,
                                 max](const QString &text) {
        bool ok;
        int line_edit_input = text.toInt(&ok, 10);  // hex == 255, ok == true


        // ignore minus sign
        if(text=="-")
        {
            return;
        }

        // handle corner case of empty input
        if (text.isEmpty())
        {
            ok              = true;
            line_edit_input = 0;
            slider->setValue(line_edit_input);
            value = line_edit_input;
        }
        else if (ok)
        {
            line_edit_input = std::clamp(line_edit_input, min, max);
            line_edit->setText(QString::number(line_edit_input));
            slider->setValue(line_edit_input);
            value = line_edit_input;
        }
        else
        {
            line_edit->setText(QString::number(value));
        }
    };

    QWidget::connect(slider, &QSlider::valueChanged, on_slider_changed);

    QWidget::connect(line_edit, &QLineEdit::textChanged, on_line_edit_changed);
}
