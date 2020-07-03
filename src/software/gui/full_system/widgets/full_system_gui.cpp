#include "software/gui/full_system/widgets/full_system_gui.h"
#include "software/gui/full_system/widgets/ai_control.h"
#include "software/gui/generic_widgets/robot_status/robot_status.h"

FullSystemGUI::FullSystemGUI(
    std::shared_ptr<ThreadSafeBuffer<WorldDrawFunction>> world_draw_functions_buffer,
    std::shared_ptr<ThreadSafeBuffer<AIDrawFunction>> ai_draw_functions_buffer,
    std::shared_ptr<ThreadSafeBuffer<PlayInfo>> play_info_buffer,
    std::shared_ptr<ThreadSafeBuffer<SensorMsg>> sensor_msg_buffer,
    std::shared_ptr<ThreadSafeBuffer<RobotStatus>> robot_status_buffer,
    std::shared_ptr<ThreadSafeBuffer<Rectangle>> view_area_buffer,
    std::shared_ptr<ThunderbotsConfig> config)
    : QMainWindow(),
      main_widget(new Ui::AutogeneratedFullSystemMainWidget()),
      update_timer(new QTimer(this)),
      world_draw_functions_buffer(world_draw_functions_buffer),
      ai_draw_functions_buffer(ai_draw_functions_buffer),
      play_info_buffer(play_info_buffer),
      sensor_msg_buffer(sensor_msg_buffer),
      robot_status_buffer(robot_status_buffer),
      view_area_buffer(view_area_buffer),
      most_recent_world_draw_function([](QGraphicsScene*){return;}),
      most_recent_ai_draw_function([](QGraphicsScene*){return;})
{
    // Create a new widget that will contain all the autogenerated
    // components defined in the .ui file. Note that because the
    // setCentralWidget call will cause this class (the Qt QMainWindow)
    // to take ownership of the widget and handle its deletion, we
    // do not need to make this a class member and delete it ourself.
    QWidget* central_widget = new QWidget(this);
    main_widget->setupUi(central_widget);
    setCentralWidget(central_widget);

    // StrongFocus means that the MainWidget will more aggressively capture focus when
    // clicked. Specifically, we do this so that when the user clicks outside of the
    // QLineEdits used for Parameters, the QLineEdit will lose focus.
    // https://www.qtcentre.org/threads/41128-Need-to-implement-in-place-line-edit-unable-to-get-lose-focus-of-QLineEdit
    setFocusPolicy(Qt::StrongFocus);

    setupRobotStatusTable(main_widget->robot_status_table_widget);
    setupAIControls(main_widget, config);

    // Update to make sure all layout changes apply nicely
    update();

    connect(update_timer, &QTimer::timeout, this, &FullSystemGUI::handleUpdate);
    update_timer->start(static_cast<int>(Duration::fromMilliseconds(UPDATE_INTERVAL_SECONDS).getMilliseconds()));
}

void FullSystemGUI::handleUpdate()
{
    draw();
    updatePlayInfo();
    updateSensorMsg();
    updateRobotStatus();
    updateDrawViewArea();
}

void FullSystemGUI::draw()
{
    if (auto world_draw_function = world_draw_functions_buffer->popLeastRecentlyAddedValue())
    {
        most_recent_world_draw_function = world_draw_function.value();
    }

    if (auto ai_draw_function = ai_draw_functions_buffer->popLeastRecentlyAddedValue())
    {
        most_recent_ai_draw_function = ai_draw_function.value();
    }

    main_widget->ai_visualization_graphics_view->clearAndDraw({most_recent_world_draw_function.getDrawFunction(), most_recent_ai_draw_function.getDrawFunction()});
}

void FullSystemGUI::updatePlayInfo()
{
    if (auto play_info = play_info_buffer->popLeastRecentlyAddedValue())
    {
        main_widget->play_info_widget->updatePlayInfo(play_info.value());
    }
}

void FullSystemGUI::updateSensorMsg()
{
    while (auto sensor_msg = sensor_msg_buffer->popLeastRecentlyAddedValue())
    {
        for (const auto& robot_msg : sensor_msg->tbots_robot_msgs())
        {
            main_widget->robot_status_table_widget->updateTbotsRobotMsg(robot_msg);
        }
    }
}

void FullSystemGUI::updateRobotStatus()
{
    while (auto robot_status = robot_status_buffer->popLeastRecentlyAddedValue())
    {
        main_widget->robot_status_table_widget->updateRobotStatus(robot_status.value());
    }
}

void FullSystemGUI::updateDrawViewArea()
{
    if (auto view_area = view_area_buffer->popLeastRecentlyAddedValue())
    {
        main_widget->ai_visualization_graphics_view->setViewArea(view_area.value());
    }
}
