#include "software/visualizer/widgets/visualizer.h"

Visualizer::Visualizer(
    std::shared_ptr<ThreadSafeBuffer<WorldDrawFunction>> world_draw_functions_buffer,
    std::shared_ptr<ThreadSafeBuffer<AIDrawFunction>> ai_draw_functions_buffer,
    std::shared_ptr<ThreadSafeBuffer<PlayInfo>> play_info_buffer,
    std::shared_ptr<ThreadSafeBuffer<RobotStatus>> robot_status_buffer)
    : QMainWindow(),
      main_widget(new MainWidget(this)),
      update_timer(new QTimer(this)),
      world_draw_functions_buffer(world_draw_functions_buffer),
      ai_draw_functions_buffer(ai_draw_functions_buffer),
      play_info_buffer(play_info_buffer),
      robot_status_buffer(robot_status_buffer),
      initial_view_area_set(false),
      update_timer_interval(Duration::fromSeconds(1.0 / 60.0))
{
    setCentralWidget(main_widget);

    connect(update_timer, &QTimer::timeout, this, &Visualizer::updateVisualizer);
    update_timer->start(update_timer_interval.getMilliseconds());
}

void Visualizer::updateVisualizer() {
    draw();
    updatePlayInfo();
    updateRobotStatus();
}

void Visualizer::draw() {
    auto world_draw_function = world_draw_functions_buffer->popLeastRecentlyAddedValue();
    if(world_draw_function) {
        most_recent_world_draw_function = world_draw_function.value();
    }

    auto ai_draw_function = ai_draw_functions_buffer->popLeastRecentlyAddedValue();
    if(ai_draw_function) {
        most_recent_ai_draw_function = ai_draw_function.value();
    }

    main_widget->draw(most_recent_world_draw_function, most_recent_ai_draw_function);

    if(!initial_view_area_set) {
        main_widget->setDrawViewAreaToSceneContents();
        initial_view_area_set = true;
    }
}

void Visualizer::updatePlayInfo() {
    auto play_info = play_info_buffer->popLeastRecentlyAddedValue();
    if(play_info) {
        main_widget->updatePlayInfo(play_info.value());
    }
}

void Visualizer::updateRobotStatus() {
    std::optional<RobotStatus> robot_status = robot_status_buffer->popLeastRecentlyAddedValue();
    while(robot_status) {
        main_widget->updateRobotStatus(robot_status.value());
        robot_status = robot_status_buffer->popLeastRecentlyAddedValue();
    }
}
