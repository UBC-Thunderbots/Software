#include "software/gui/standalone_simulator/widgets/standalone_simulator_draw_function_visualizer.h"

#include <QtWidgets/QMenu>

#include "software/gui/drawing/ball.h"
#include "software/gui/geometry_conversion.h"
#include "software/simulation/simulator.h"


StandaloneSimulatorDrawFunctionVisualizer::StandaloneSimulatorDrawFunctionVisualizer(
    QWidget* parent)
    : DrawFunctionVisualizer(parent)
{
    // Let mouseMoveEvents be triggered even if a mouse button is not pressed
    this->setMouseTracking(true);
}

void StandaloneSimulatorDrawFunctionVisualizer::setStandaloneSimulator(
    std::shared_ptr<StandaloneSimulator> simulator)
{
    standalone_simulator = simulator;
}

void StandaloneSimulatorDrawFunctionVisualizer::mousePressEvent(QMouseEvent* event)
{
    // If Ctrl is pressed, place the ball where the user clicks
    if (event->modifiers() & Qt::ControlModifier && standalone_simulator)
    {
        ctrl_clicked  = true;
        initial_point = createPoint(mapToScene(event->pos()));
        final_point   = createPoint(mapToScene(event->pos()));
        standalone_simulator->setBallState(BallState(initial_point, Vector(0, 0)));
    }
    else if (event->modifiers() & Qt::ShiftModifier && standalone_simulator)
    {
        Point point_in_scene = createPoint(mapToScene(event->pos()));
        robot                = standalone_simulator->getRobotAtPosition(point_in_scene);
    }
    else
    {
        robot.reset();
        DrawFunctionVisualizer::mousePressEvent(event);
    }
}

void StandaloneSimulatorDrawFunctionVisualizer::mouseReleaseEvent(QMouseEvent* event)
{
    robot.reset();
    if (ctrl_clicked)
    {
        final_point = createPoint(mapToScene(event->pos()));
        standalone_simulator->setBallState(
            BallState(initial_point, initial_point - final_point));
        initial_point = Point(0, 0);
        final_point   = Point(0, 0);
        ctrl_clicked  = false;
    }

    DrawFunctionVisualizer::mouseReleaseEvent(event);
}

void StandaloneSimulatorDrawFunctionVisualizer::mouseMoveEvent(QMouseEvent* event)
{
    auto physics_robot = robot.lock();
    if (physics_robot && standalone_simulator)
    {
        Point point_in_scene = createPoint(mapToScene(event->pos()));
        physics_robot->setPosition(point_in_scene);
    }
    if (ctrl_clicked && standalone_simulator)
    {
        final_point = createPoint(mapToScene(event->pos()));
    }
    DrawFunctionVisualizer::mouseMoveEvent(event);
}

WorldDrawFunction StandaloneSimulatorDrawFunctionVisualizer::getDrawBallVelocityFunction()
{
    auto draw_function = [this](QGraphicsScene* scene) {
        drawBallVelocity(scene, initial_point, initial_point - final_point,
                         ball_speed_slow_color, ball_speed_fast_color);
    };
    return WorldDrawFunction(draw_function);
}

void StandaloneSimulatorDrawFunctionVisualizer::contextMenuEvent(QContextMenuEvent* event)
{
    Point point_in_scene    = createPoint(mapToScene(event->pos()));
    auto robot_under_cursor = standalone_simulator->getRobotAtPosition(point_in_scene);

    QMenu menu(this);
    menu.addAction("Reset View",
                   [this]() { DrawFunctionVisualizer::setViewArea(lastViewArea); });
    menu.addAction("Place Ball Here", [&]() {
        standalone_simulator->setBallState(BallState{
            .position_ = point_in_scene, .velocity_ = Vector(0, 0), .height_ = 0});
    });
    menu.addAction("Add Yellow Robot Here",
                   [&]() { standalone_simulator->addYellowRobot(point_in_scene); });
    menu.addAction("Add Blue Robot Here",
                   [&]() { standalone_simulator->addBlueRobot(point_in_scene); });
    if (auto physics_robot = robot_under_cursor.lock())
    {
        menu.addAction("Move Robot", [&]() { robot = robot_under_cursor; });
        menu.addAction("Remove Robot",
                       [&]() { standalone_simulator->removeRobot(robot_under_cursor); });
    }

    menu.exec(event->globalPos());
}
