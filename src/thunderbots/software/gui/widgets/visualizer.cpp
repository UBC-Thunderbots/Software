#include "software/gui/widgets/visualizer.h"

Visualizer::Visualizer() : QMainWindow(), main_widget(new MainWidget(this))
{
    setCentralWidget(main_widget);
}

Visualizer::~Visualizer()
{
    delete main_widget;
}

void Visualizer::draw(WorldDrawFunction world_draw_function,
                      AIDrawFunction ai_draw_function)
{
    main_widget->draw(world_draw_function, ai_draw_function);
}

void Visualizer::updatePlayInfo(const PlayInfo& play_info)
{
    main_widget->updatePlayInfo(play_info);
}

void Visualizer::updateRobotStatus(const RobotStatus& robot_status)
{
    main_widget->updateRobotStatus(robot_status);
}
