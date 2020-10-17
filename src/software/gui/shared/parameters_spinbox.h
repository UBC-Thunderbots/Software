#pragma once

#include "software/parameter/parameter.h"
#include <QtWidgets/QSpinBox>
#include <memory>

/**
 * Helper function for setting the value of the spinbox
 *
 * @param spin_box
 * @param parameter
 */
void setup__SpinBox(QSpinBox* spin_box, std::shared_ptr < Parameter < int >> & parameter);