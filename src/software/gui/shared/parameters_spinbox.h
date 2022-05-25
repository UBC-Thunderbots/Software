#pragma once

#include <QtWidgets/QSpinBox>
#include <memory>

#include "shared/parameter/parameter.h"

/**
 * Helper function for setting the value of the spinbox
 *
 * @param spin_box The spinbox to setup
 * @param parameter The parameter whose value is changed
 */
void setupSpinBox(QSpinBox* spin_box, std::shared_ptr<NumericParameter<int>>& parameter);
