#pragma once

#include <QtWidgets/QGraphicsScene>
#include <QtWidgets/QOpenGLWidget>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QWidget>

// This include is autogenerated by the .ui file in the same folder
// The generated version will be names 'ui_<filename>.h'
#include "software/ai/hl/stp/play_info.h"
#include "software/backend/robot_status.h"
#include "software/visualizer/drawing/draw_functions.h"
#include "software/visualizer/ui/ui_main_widget.h"
#include "software/visualizer/widgets/ai_control.h"
#include "software/visualizer/widgets/parameters.h"
#include "software/visualizer/widgets/robot_status.h"
#include "software/visualizer/widgets/world_view.h"


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
     * @param world_draw_function The function that tells the Visualizer how to draw the
     * World state
     * @param ai_draw_function The function that tells the Visualizer how to draw the AI
     * state
     */
    void draw(WorldDrawFunction world_draw_function, AIDrawFunction ai_draw_function);

    /**
     * Updates and displays newly provided PlayInfo
     *
     * @param play_info The new PlayInfo to display
     */
    void updatePlayInfo(const PlayInfo& play_info);

    /**
     * Updates and displays the newly provided RobotStatus.
     *
     * @param robot_status The new robot status to display
     */
    void updateRobotStatus(const RobotStatus& robot_status);

   private:
    // Unfortunately Qt uses raw pointers
    // MAKE SURE TO DELETE ANY RAW POINTERS IN THE DESTRUCTOR
    Ui::AutoGeneratedMainWidget* main_widget;
    bool first_draw_call;
    QGraphicsScene* scene;
    QOpenGLWidget* glWidget;
};
