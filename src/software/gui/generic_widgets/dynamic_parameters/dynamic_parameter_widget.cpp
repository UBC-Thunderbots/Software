#include "software/gui/generic_widgets/dynamic_parameters/dynamic_parameter_widget.h"

#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include <limits>

#include "software/gui/shared/parameters_spinbox.h"
#include "software/logger/logger.h"
#include "software/util/variant_visitor/variant_visitor.h"

DynamicParameterWidget::DynamicParameterWidget(QWidget* parent) : QScrollArea(parent)
{
    params_widget                     = new QWidget(this);
    QVBoxLayout* params_widget_layout = new QVBoxLayout(params_widget);
    params_widget->setLayout(params_widget_layout);
}

void DynamicParameterWidget::setupParameters(std::shared_ptr<Config> config)
{
    if (!params_widget->layout())
    {
        throw std::invalid_argument("Cannot setup a parameter widget without a layout");
    }

    for (auto mutable_parameter : config->getMutableParameterList())
    {
        std::visit(
            overload{[&](std::shared_ptr<Parameter<bool>> param) {
                         QWidget* bool_param_widget = createBooleanParameter(param);
                         bool_param_widget->setParent(params_widget);
                         params_widget->layout()->addWidget(bool_param_widget);
                     },
                     [&](std::shared_ptr<Parameter<std::string>> param) {
                         QWidget* string_param_widget = createStringParameter(param);
                         string_param_widget->setParent(params_widget);
                         params_widget->layout()->addWidget(string_param_widget);
                     },
                     [&](std::shared_ptr<NumericParameter<double>> param) {
                         QWidget* double_param_widget = createDoubleParameter(param);
                         double_param_widget->setParent(params_widget);
                         params_widget->layout()->addWidget(double_param_widget);
                     },
                     [&](std::shared_ptr<NumericParameter<int>> param) {
                         QWidget* int_param_widget = createIntegerParameter(param);
                         int_param_widget->setParent(params_widget);
                         params_widget->layout()->addWidget(int_param_widget);
                     },
                     [&](std::shared_ptr<Config> config) {
                         QWidget* config_label_widget = createConfigLabel(config);
                         config_label_widget->setParent(params_widget);
                         params_widget->layout()->addWidget(config_label_widget);
                         setupParameters(config);
                     }},
            mutable_parameter);
    }
    setWidgetResizable(true);
    setWidget(params_widget);
}

QWidget* DynamicParameterWidget::createConfigLabel(std::shared_ptr<Config> config)
{
    QWidget* widget     = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(widget);

    QLabel* label = new QLabel(widget);
    label->setText(QString::fromStdString(config->name()));
    label->setText(QString::fromStdString("<u>" + config->name() + "</u>"));
    layout->addWidget(label);
    widget->setLayout(layout);

    return widget;
}

QWidget* DynamicParameterWidget::createBooleanParameter(
    std::shared_ptr<Parameter<bool>> parameter)
{
    QWidget* widget     = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(widget);

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
    std::shared_ptr<NumericParameter<int>> parameter)
{
    QWidget* widget     = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(widget);

    QLabel* label = new QLabel(widget);
    label->setText(QString::fromStdString(parameter->name()));
    QSpinBox* spinbox = new QSpinBox(widget);

    spinbox->setRange(parameter->getMin(), parameter->getMax());
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

    setupSpinBox(spinbox, parameter);

    widget->setLayout(layout);

    return widget;
}

QWidget* DynamicParameterWidget::createDoubleParameter(
    std::shared_ptr<NumericParameter<double>> parameter)
{
    QWidget* widget              = new QWidget();
    QVBoxLayout* vertical_layout = new QVBoxLayout(widget);

    QLabel* label = new QLabel(widget);
    label->setText(QString::fromStdString(parameter->name()));
    vertical_layout->addWidget(label);

    QHBoxLayout* horizontal_layout = new QHBoxLayout();
    horizontal_layout->setSpacing(6);

    QSlider* param_slider = new QSlider(widget);
    param_slider->setOrientation(Qt::Horizontal);
    horizontal_layout->addWidget(param_slider);

    QLineEdit* param_line_edit = new QLineEdit(widget);
    param_line_edit->setMaximumWidth(60);
    horizontal_layout->addWidget(param_line_edit);
    vertical_layout->addLayout(horizontal_layout);

    widget->setLayout(vertical_layout);

    auto on_slider_value_changed = [parameter](double new_value) {
        LOG(INFO) << "Value for " << parameter->name() << " (double param) changed to "
                  << new_value << std::endl;
        parameter->setValue(new_value);
    };

    auto on_parameter_value_changed =
        setupSliderLineEdit(param_line_edit, param_slider, on_slider_value_changed,
                            parameter->getMin(), parameter->getMax(), 100);

    parameter->registerCallbackFunction(on_parameter_value_changed);
    on_parameter_value_changed(parameter->value());

    return widget;
}

QWidget* DynamicParameterWidget::createStringParameter(
    std::shared_ptr<Parameter<std::string>> parameter)
{
    QWidget* widget     = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(widget);

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
