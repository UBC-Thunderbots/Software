#pragma once

#include <QMainWindow>

#include "gui/main_widget.h"
#include "ai/world/world.h"

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
     * Draws all the AI information we want to display in the Visualizer. This includes visualizing the state of
     * the world as well as drawing the AI state we want to show, like planned navigator paths.
     *
     * @param world The world to draw
     */
    void drawAI(World world, QGraphicsScene* scene);


   private:
    // Unfortunately Qt uses raw pointers
    // MAKE SURE TO DELETE ANY RAW POINTERS IN THE DESTRUCTOR
    MainWidget* main_widget;
};
