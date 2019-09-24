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

void ThunderbotsVisualizer::drawAI(World world)
{
    main_widget->drawAI(world);
}

void ThunderbotsVisualizer::drawAITest(DrawFunction draw_function) {
    main_widget->drawAITest(draw_function);
}
