#pragma once

#include <QtWidgets/QScrollArea>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "shared/parameter/enumerated_parameter.h"
#include "shared/parameter/numeric_parameter.h"
#include "shared/parameter/parameter.h"
#include "software/gui/generic_widgets/slider/slider.h"

/**
 * This widget displays all our DynamicParameters and allows the user to
 * change their values
 */
class DynamicParameterWidget : public QScrollArea
{
    Q_OBJECT

   public:
    explicit DynamicParameterWidget(QWidget* parent = nullptr);

    /**
     * A helper function to recursively setup all parameters and sub-configs of the given
     * config.
     *
     * @param config The config to setup
     */
    void setupParameters(std::shared_ptr<Config> config);

   private:
    /**
     * Creates a widget that contains a label for a Config
     *
     * @param config The Config to make a label for
     *
     * @return A pointer to the QWidget that will be used to show the label
     */
    QWidget* createConfigLabel(std::shared_ptr<Config> config);

   private:
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
    static QWidget* createIntegerParameter(
        std::shared_ptr<NumericParameter<int>> parameter);

    /**
     * Creates a widget that contains the components necessary to display and control a
     * double Parameter for the AI
     *
     * @param parameter The double parameter to create a widget for
     * @return A pointer to the QWidget that will be used to control the given parameter
     */
    static QWidget* createDoubleParameter(
        std::shared_ptr<NumericParameter<double>> parameter);

    /**
     * Creates a widget that contains the components necessary to display and control a
     * string Parameter for the AI
     *
     * @param parameter The string parameter to create a widget for
     * @return A pointer to the QWidget that will be used to control the given parameter
     */
    static QWidget* createStringParameter(
        std::shared_ptr<Parameter<std::string>> parameter);

    QWidget* params_widget;
};
