#pragma once

#include <QMainWindow>
#include <QTimer>

#include "software/gui/main_widget.h"
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
    void drawAI(World world);

   private:
    MainWidget* main_widget;
    QTimer* draw_AI_timer;
};
