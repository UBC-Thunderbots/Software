#include <QtWidgets/QSpinBox>
#include <memory>

#include "shared/parameter/numeric_parameter.h"
#include "shared/parameter/parameter.h"

void setupSpinBox(QSpinBox* spin_box, std::shared_ptr<NumericParameter<int>>& parameter)
{
    auto on_parameter_value_changed = [spin_box](int new_value) {
        // We block signals while setting the value of the spinbox so that we don't
        // trigger the `on_spinbox_value_changed` function, which would set the
        // parameter value again and deadlock on the parameter's internal mutex
        spin_box->blockSignals(true);
        spin_box->setValue(new_value);
        spin_box->blockSignals(false);
    };
    parameter->registerCallbackFunction(on_parameter_value_changed);
}
