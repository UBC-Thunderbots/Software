#pragma once

#include <QtWidgets/QMainWindow>

#include "software/gui/widgets/main_widget.h"
#include "software/gui/drawing/draw_functions.h"

using FOOBAR = std::vector<std::pair<std::string, Duration>>;
/**
 * This is the main window / application object for the thunderbots visualizer
 * It combines a menu with our "main widget"
 */
class ThunderbotsVisualizer : public QMainWindow
{
    Q_OBJECT

   public:
    explicit ThunderbotsVisualizer();
    ~ThunderbotsVisualizer();
   public slots:
    /**
     * Draws all the AI information we want to display in the Visualizer. This includes
     * visualizing the state of the world as well as drawing the AI state we want to show,
     * like planned navigator paths.
     *
     * @param world The world to draw
     */
    void draw(WorldDrawFunction world_draw_function, AIDrawFunction ai_draw_function);
    void updatePlayInfo(PlayInfo play_info);
    void updateRobotStatus(FOOBAR robot_status);

   private:
    // Unfortunately Qt uses raw pointers
    // MAKE SURE TO DELETE ANY RAW POINTERS IN THE DESTRUCTOR
    MainWidget* main_widget;
};
