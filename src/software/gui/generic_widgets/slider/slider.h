#pragma once

#include <QtWidgets/QLineEdit>
#include <QtWidgets/QSlider>
#include <QtWidgets/QWidget>

/**
 * Sets up slider and line edit linkage and modifies the value as input changes
 * Implements input checking and clamping behaviour
 *
 * @param line_edit QLineEdit to read and modify
 * @param slider QSlider to read and modify
 * @param value_changed_callback callback to call with new values
 * @param min min value for value
 * @param max max value for value
 * @param slider_step_size how much the line_edit value should be scaled up to display on
 * slider
 *
 * @return function to call to set the slider and line edit to a new value
 */
std::function<void(double)> setupSliderLineEdit(
    QLineEdit *line_edit, QSlider *slider,
    std::function<void(double)> value_changed_callback, double min, double max,
    double slider_step_size);
