#pragma once

#include <QtWidgets/QWidget>
#include <QtWidgets/QVBoxLayout>

#include "software/parameter/dynamic_parameters.h"
#include "software/parameter/parameter.h"

/**
 * TODO: comment
 */
class DynamicParameterWidget : public QWidget {
    Q_OBJECT

public:
    // TODO: comment
    // TODO: take the params as a parameter
    explicit DynamicParameterWidget(QWidget* parent = 0);

private:
    // TODO: comment
    void setupParametersHelper(QWidget* params_widget, QVBoxLayout* layout, MutableParameterVariant param_var);

    /**
     * Creates a widget that contains the components necessary to display and control a
     * boolean Parameter for the AI
     *
     * @param parameter The boolean parameter to create a widget for
     * @return A pointer to the QWidget that will be used to control the given parameter
     */
    static QWidget* createBooleanParameter(std::shared_ptr<Parameter<bool>> parameter);

    /**
     * Creates a widget that contains the components necessary to display and control an
     * integer Parameter for the AI
     *
     * @param parameter The integer parameter to create a widget for
     * @return A pointer to the QWidget that will be used to control the given parameter
     */
    static QWidget* createIntegerParameter(std::shared_ptr<Parameter<int>> parameter);

    /**
     * Creates a widget that contains the components necessary to display and control a
     * double Parameter for the AI
     *
     * @param parameter The double parameter to create a widget for
     * @return A pointer to the QWidget that will be used to control the given parameter
     */
    static QWidget* createDoubleParameter(std::shared_ptr<Parameter<double>> parameter);

    /**
     * Creates a widget that contains the components necessary to display and control a
     * string Parameter for the AI
     *
     * @param parameter The string parameter to create a widget for
     * @return A pointer to the QWidget that will be used to control the given parameter
     */
    static QWidget* createStringParameter(std::shared_ptr<Parameter<std::string>> parameter);
};
