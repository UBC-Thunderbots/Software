#include "software/gui/main_window.h"

ThunderbotsVisualizer::ThunderbotsVisualizer() :
        QMainWindow(),
        main_widget(new MainWidget(this)),
        draw_AI_timer(new QTimer(this))
        {
    setCentralWidget(main_widget);

    connect(draw_AI_timer, &QTimer::timeout, main_widget, &MainWidget::drawAI);
    draw_AI_timer->start(33);
}

ThunderbotsVisualizer::~ThunderbotsVisualizer() {
    delete main_widget;
    delete draw_AI_timer;
}
