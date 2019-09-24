#pragma once

#include <QtWidgets/QGraphicsScene>
#include <QtWidgets/QOpenGLWidget>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QWidget>

#include "software/ai/world/world.h"
#include "software/gui/widgets/zoomable_qgraphics_view.h"
#include "software/util/parameter/parameter.h"
#include "software/gui/drawing/typedefs.h"

// Forward declare the name of the top-level GUI class defined in main_widget.ui
namespace Ui
{
    class AutoGeneratedMainWidget;
}

/**
 * This class acts as a wrapper widget for all the autogenerated components
 * defined in main_widget.ui
 */
class MainWidget : public QWidget
{
    Q_OBJECT

   public:
    explicit MainWidget(QWidget* parent = nullptr);
    ~MainWidget();

   public slots:

    /**
     * Draws all the AI information we want to display in the Visualizer. This includes
     * visualizing the state of the world as well as drawing the AI state we want to show,
     * like planned navigator paths.
     *
     * @param world The world to draw
     */
    void drawAI(World world);
    void drawAITest(DrawFunction draw_function);

    // TODO: comment
    void updateRobotStatusMessages();

   private:
    /**
     * Sets up the SceneView which draws the AI's view of the world, such as robot
     * positions, ball velocity, etc.
     * @param view The view to display the QGraphicsScene with
     * @param scene The QGraphicsScene that will be dislpayed by the view
     * @param gl_widget A QOpenGLWidget that can be used to help display items in the view
     */
    void setupSceneView(QGraphicsView* view, QGraphicsScene* scene,
                        QOpenGLWidget* gl_widget);

    /**
     * Sets up the status table that displays robot status
     * @param table The QTableWidget that will contain the status information
     */
    void setupStatusTable(QTableWidget* table);

    /**
     * Sets up all the widgets that will be used to control the AI, such as starting or
     * stopping it, or choosing the colour of the friendly team
     */
    void setupAIControls();

    /**
     * Sets up the buttons that will start and top the AI
     */
    void setupAIStartAndStopButtons();

    /**
     * Sets up the ComboBox widget that will be used to select the friendly team colour
     */
    void setupTeamColourComboBox();

    /**
     * Sets up the ComboBox widget that will be used to select the side the friendly team
     * is defending
     */
    void setupDefendingSideComboBox();

    /**
     * Sets up the ComboBox widget that will be used to override the AI GameState
     */
    void setupGameStateOverrideComboBox();

    /**
     * Sets up the ComboBox widget that will be used to override the current AI Play
     */
    void setupPlayOverrideComboBox();

    /**
     * Sets up the Parameters tab that contains all the tuneable parameters for the AI
     */
    void setupParametersTab();

    /**
     * Creates a widget that contains the components necessary to display and control a
     * boolean Parameter for the AI
     * @param parameter The boolean parameter to create a widget for
     * @return A pointer to the QWidget that will be used to control the given parameter
     */
    QWidget* createBooleanParameter(Parameter<bool>* parameter);

    /**
     * Creates a widget that contains the components necessary to display and control an
     * integer Parameter for the AI
     * @param parameter The integer parameter to create a widget for
     * @return A pointer to the QWidget that will be used to control the given parameter
     */
    QWidget* createIntegerParameter(Parameter<int>* parameter);

    /**
     * Creates a widget that contains the components necessary to display and control a
     * double Parameter for the AI
     * @param parameter The double parameter to create a widget for
     * @return A pointer to the QWidget that will be used to control the given parameter
     */
    QWidget* createDoubleParameter(Parameter<double>* parameter);

    /**
     * Creates a widget that contains the components necessary to display and control a
     * string Parameter for the AI
     * @param parameter The string parameter to create a widget for
     * @return A pointer to the QWidget that will be used to control the given parameter
     */
    QWidget* createStringParameter(Parameter<std::string>* parameter);

    // TODO: comment
    void setRobotStatus(QTableWidget* table,
                        std::vector<std::string> robot_status_messages);

    bool first_draw_call;
    // Unfortunately Qt uses raw pointers
    // MAKE SURE TO DELETE ANY RAW POINTERS IN THE DESTRUCTOR
    Ui::AutoGeneratedMainWidget* main_widget;
    QGraphicsScene* scene;
    QOpenGLWidget* glWidget;
};
