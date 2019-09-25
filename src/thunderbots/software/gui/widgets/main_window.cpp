#include "software/gui/widgets/main_window.h"

ThunderbotsVisualizer::ThunderbotsVisualizer()
    : QMainWindow(), main_widget(new MainWidget(this))
{
    setCentralWidget(main_widget);
}

ThunderbotsVisualizer::~ThunderbotsVisualizer()
{
    delete main_widget;
}

void ThunderbotsVisualizer::draw(WorldDrawFunction world_draw_function, AIDrawFunction ai_draw_function) {
    main_widget->draw(world_draw_function, ai_draw_function);
}
