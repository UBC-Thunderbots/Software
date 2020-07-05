#include "software/gui/generic_widgets/dynamic_parameters/dynamic_parameter_widget.h"

#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

#include "software/logger/logger.h"
#include "software/util/variant_visitor/variant_visitor.h"

DynamicParameterWidget::DynamicParameterWidget(QWidget* parent) : QScrollArea(parent)
{
    QWidget* params_widget            = new QWidget(this);
    QVBoxLayout* params_widget_layout = new QVBoxLayout(params_widget);
    params_widget->setLayout(params_widget_layout);

    setupParametersHelper(params_widget, MutableDynamicParameters);

    setWidget(params_widget);
    setWidgetResizable(true);
}

void DynamicParameterWidget::setupParametersHelper(QWidget* params_widget,
                                                   std::shared_ptr<Config> config)
{
    if (!params_widget->layout())
    {
        throw std::invalid_argument("Cannot setup a parameter widget without a layout");
    }

    for (auto mutable_parameter : config->getMutableParameterList())
    {
        std::visit(
            overload{[&](std::shared_ptr<Parameter<int>> param) {
                         QWidget* int_param_widget = createIntegerParameter(param);
                         int_param_widget->setParent(params_widget);
                         params_widget->layout()->addWidget(int_param_widget);
                     },
                     [&](std::shared_ptr<Parameter<bool>> param) {
                         QWidget* bool_param_widget = createBooleanParameter(param);
                         bool_param_widget->setParent(params_widget);
                         params_widget->layout()->addWidget(bool_param_widget);
                     },
                     [&](std::shared_ptr<Parameter<std::string>> param) {
                         QWidget* string_param_widget = createStringParameter(param);
                         string_param_widget->setParent(params_widget);
                         params_widget->layout()->addWidget(string_param_widget);
                     },
                     [&](std::shared_ptr<Parameter<double>> param) {
                         QWidget* double_param_widget = createDoubleParameter(param);
                         double_param_widget->setParent(params_widget);
                         params_widget->layout()->addWidget(double_param_widget);
                     },
                     [&](std::shared_ptr<Config> config_) {
                         setupParametersHelper(params_widget, config_);
                     }},
            mutable_parameter);
    }
}

QWidget* DynamicParameterWidget::createBooleanParameter(
    std::shared_ptr<Parameter<bool>> parameter)
{
    QWidget* widget     = new QWidget();
    QHBoxLayout* layout = new QHBoxLayout(widget);

    QLabel* label = new QLabel(widget);
    label->setText(QString::fromStdString(parameter->name()));
    QCheckBox* checkbox = new QCheckBox(widget);
    checkbox->setChecked(parameter->value());

    layout->addWidget(label);
    layout->addWidget(checkbox);

    auto on_checkbox_value_changed = [parameter, checkbox]() {
        LOG(INFO) << "Value for boolean param " << parameter->name() << " changed to "
                  << checkbox->isChecked() << std::endl;
        parameter->setValue(checkbox->isChecked());
    };
    QWidget::connect(checkbox, &QCheckBox::stateChanged, on_checkbox_value_changed);

    auto on_parameter_value_changed = [checkbox](bool new_value) {
        // We block signals while setting the state of the checkbox so that we don't
        // trigger the `on_checkbox_value_changed` function, which would set the
        // parameter value again and deadlock on the parameter's internal mutex
        checkbox->blockSignals(true);
        checkbox->setChecked(new_value);
        checkbox->blockSignals(false);
    };
    parameter->registerCallbackFunction(on_parameter_value_changed);

    widget->setLayout(layout);

    return widget;
}

QWidget* DynamicParameterWidget::createIntegerParameter(
    std::shared_ptr<Parameter<int>> parameter)
{
    QWidget* widget     = new QWidget();
    QHBoxLayout* layout = new QHBoxLayout(widget);

    QLabel* label = new QLabel(widget);
    label->setText(QString::fromStdString(parameter->name()));
    QSpinBox* spinbox = new QSpinBox(widget);
    // TODO: Get range from parameter
    spinbox->setRange(0, 100);
    spinbox->setValue(parameter->value());

    layout->addWidget(label);
    layout->addWidget(spinbox);

    auto on_spinbox_value_changed = [parameter, spinbox]() {
        LOG(INFO) << "Value for integer param " << parameter->name() << " changed to "
                  << spinbox->value() << std::endl;
        parameter->setValue(spinbox->value());
    };
    // QSpinBox has 2 "valueChanged" signals that each provide different info (string vs
    // int), so we need to static_cast to specify the integer version
    QWidget::connect(spinbox,
                     static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                     on_spinbox_value_changed);

    auto on_parameter_value_changed = [spinbox](int new_value) {
        // We block signals while setting the value of the spinbox so that we don't
        // trigger the `on_spinbox_value_changed` function, which would set the
        // parameter value again and deadlock on the parameter's internal mutex
        spinbox->blockSignals(true);
        spinbox->setValue(new_value);
        spinbox->blockSignals(false);
    };
    parameter->registerCallbackFunction(on_parameter_value_changed);

    widget->setLayout(layout);

    return widget;
}

QWidget* DynamicParameterWidget::createDoubleParameter(
    std::shared_ptr<Parameter<double>> parameter)
{
    QWidget* widget     = new QWidget();
    QHBoxLayout* layout = new QHBoxLayout(widget);

    QLabel* label = new QLabel(widget);
    label->setText(QString::fromStdString(parameter->name()));
    QDoubleSpinBox* spinbox = new QDoubleSpinBox(widget);
    // TODO: Get range from parameter
    spinbox->setRange(0, 100);
    spinbox->setValue(parameter->value());
    spinbox->setSingleStep(0.05);

    layout->addWidget(label);
    layout->addWidget(spinbox);

    auto on_spinbox_value_changed = [parameter, spinbox]() {
        LOG(INFO) << "Value for double param " << parameter->name() << " changed to "
                  << spinbox->value() << std::endl;
        parameter->setValue(spinbox->value());
    };
    // QDoubleSpinBox has 2 "valueChanged" signals that each provide different info
    // (string vs int), so we need to static_cast to specify the integer version
    QWidget::connect(
        spinbox,
        static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
        on_spinbox_value_changed);

    auto on_parameter_value_changed = [spinbox](int new_value) {
        // We block signals while setting the value of the spinbox so that we don't
        // trigger the `on_spinbox_value_changed` function, which would set the
        // parameter value again and deadlock on the parameter's internal mutex
        spinbox->blockSignals(true);
        spinbox->setValue(new_value);
        spinbox->blockSignals(false);
    };
    parameter->registerCallbackFunction(on_parameter_value_changed);

    widget->setLayout(layout);

    return widget;
}

QWidget* DynamicParameterWidget::createStringParameter(
    std::shared_ptr<Parameter<std::string>> parameter)
{
    QWidget* widget     = new QWidget();
    QHBoxLayout* layout = new QHBoxLayout(widget);

    QLabel* label = new QLabel(widget);
    label->setText(QString::fromStdString(parameter->name()));
    QLineEdit* line_edit = new QLineEdit(widget);
    line_edit->setText(QString::fromStdString(parameter->value()));

    layout->addWidget(label);
    layout->addWidget(line_edit);

    auto on_line_edit_text_changed = [parameter, line_edit]() {
        LOG(INFO) << "Value for string param " << parameter->name() << " changed to "
                  << line_edit->text().toStdString() << std::endl;
        parameter->setValue(line_edit->text().toStdString());
    };
    // This event will only fire when "Enter" is pressed or the LineEdit loses focus,
    // rather than every time a character changes in the LineEdit.
    // https://doc.qt.io/archives/qt-4.8/qlineedit.html#editingFinished
    QWidget::connect(line_edit, &QLineEdit::editingFinished, on_line_edit_text_changed);

    auto on_parameter_value_changed = [line_edit](std::string new_value) {
        // We block signals while setting the text of the LineEdit so that we don't
        // trigger the `on_line_edit_text_changed` function, which would set the
        // parameter value again and deadlock on the parameter's internal mutex
        line_edit->blockSignals(true);
        line_edit->setText(QString::fromStdString(new_value));
        line_edit->blockSignals(false);
    };
    parameter->registerCallbackFunction(on_parameter_value_changed);

    widget->setLayout(layout);

    return widget;
}
