#include "software/gui/standalone_simulator/widgets/standalone_simulator_draw_function_visualizer.h"

#include "software/gui/geometry_conversion.h"
#include <QtWidgets/QMenu>

StandaloneSimulatorDrawFunctionVisualizer::StandaloneSimulatorDrawFunctionVisualizer(
    QWidget* parent)
    : DrawFunctionVisualizer(parent)
{
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
        Point point_in_scene = createPoint(mapToScene(event->pos()));
        standalone_simulator->setBallState(BallState(point_in_scene, Vector(0, 0)));
    }
    else if (event->modifiers() & Qt::ShiftModifier && standalone_simulator)
    {
        Point point_in_scene = createPoint(mapToScene(event->pos()));
        robot                = standalone_simulator->getRobotAtPosition(point_in_scene);
    }
    else
    {
        DrawFunctionVisualizer::mousePressEvent(event);
    }
}

void StandaloneSimulatorDrawFunctionVisualizer::mouseReleaseEvent(QMouseEvent* event)
{
    robot.reset();
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
    DrawFunctionVisualizer::mouseMoveEvent(event);
}

void StandaloneSimulatorDrawFunctionVisualizer::contextMenuEvent(QContextMenuEvent *event) {
    Point point_in_scene = createPoint(mapToScene(event->pos()));

    QMenu menu(this);
    menu.addAction("Add Yellow Robot Here", [&]() {
        standalone_simulator->addYellowRobot(point_in_scene);
    });
    menu.addAction("Add Blue Robot Here", [&]() {
        standalone_simulator->addBlueRobot(point_in_scene);
    });

    menu.exec(event->globalPos());
}
