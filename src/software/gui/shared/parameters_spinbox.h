#pragma once

#include <QtWidgets/QSpinBox>
#include <memory>

#include "software/parameter/parameter.h"

/**
 * Helper function for setting the value of the spinbox
 *
 * @param spin_box
 * @param parameter
 */
void setupSpinBox(QSpinBox* spin_box, std::shared_ptr<Parameter<int>>& parameter);