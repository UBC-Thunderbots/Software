#pragma once

#include <QtWidgets/QScrollArea>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

#include "software/parameter/dynamic_parameters.h"
#include "software/parameter/parameter.h"

/**
 * This widget displays all our DynamicParameters and allows the user to
 * change their values
 */
class DynamicParameterWidget : public QScrollArea
{
    Q_OBJECT

   public:
    explicit DynamicParameterWidget(QWidget* parent = 0);

   private:
    /**
     * A helper function to recursively setup all parameters and sub-configs of the given
     * config.
     *
     * @pre the params_widget must have an initialized layout (ie. the layout must
     * not be null)
     *
     * @param params_widget The widget to add parameters to
     * @param config The config to setup
     */
    void setupParametersHelper(QWidget* params_widget, std::shared_ptr<Config> config);

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
    static QWidget* createStringParameter(
        std::shared_ptr<Parameter<std::string>> parameter);
};
