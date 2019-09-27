#pragma once

#include <QtWidgets/QMainWindow>

#include "software/ai/hl/stp/play_info.h"
#include "software/backend/robot_status.h"
#include "software/gui/drawing/draw_functions.h"
#include "software/gui/widgets/main_widget.h"

/**
 * This is the main window / application object for the visualizer.
 * Currently it wraps the "main_widget" which is what actually contains
 * all the content
 */
class Visualizer : public QMainWindow
{
    Q_OBJECT

   public:
    explicit Visualizer();
    ~Visualizer();

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
    MainWidget* main_widget;
};
