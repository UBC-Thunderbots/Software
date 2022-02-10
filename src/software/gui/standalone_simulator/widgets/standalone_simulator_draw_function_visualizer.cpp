#include "software/gui/standalone_simulator/widgets/standalone_simulator_draw_function_visualizer.h"

#include <QtWidgets/QMenu>

#include "software/gui/drawing/ball.h"
#include "software/gui/geometry_conversion.h"
#include "software/simulation/simulator.h"


StandaloneSimulatorDrawFunctionVisualizer::StandaloneSimulatorDrawFunctionVisualizer(
    QWidget* parent)
    : DrawFunctionVisualizer(parent), shift_clicked(false), ctrl_clicked(false)
{
    // Let mouseMoveEvents be triggered even if a mouse button is not pressed
    this->setMouseTracking(true);
}

void StandaloneSimulatorDrawFunctionVisualizer::setStandaloneSimulator(
    std::shared_ptr<StandaloneSimulator> simulator)
{
    standalone_simulator = simulator;
}

void StandaloneSimulatorDrawFunctionVisualizer::keyPressEvent(QKeyEvent* event)
{
    if (event->key() == Qt::Key_R)
    {
        r_clicked = true;
    }
}

void StandaloneSimulatorDrawFunctionVisualizer::resetFlags()
{
    initial_click_point = Point(0, 0);
    final_click_point   = Point(0, 0);
    ctrl_clicked        = false;
    shift_clicked       = false;
    r_clicked           = false;
}

void StandaloneSimulatorDrawFunctionVisualizer::mousePressEvent(QMouseEvent* event)
{
    // If Ctrl is pressed, place the ball where the user clicks
    if (event->modifiers() & Qt::ControlModifier && standalone_simulator)
    {
        ctrl_clicked        = true;
        initial_click_point = createPoint(mapToScene(event->pos()));
        final_click_point   = createPoint(mapToScene(event->pos()));
        standalone_simulator->setBallState(BallState(initial_click_point, Vector(0, 0)));
    }
    else if (event->modifiers() & Qt::ShiftModifier && standalone_simulator)
    {
        shift_clicked        = true;
        Point point_in_scene = createPoint(mapToScene(event->pos()));
        robot                = standalone_simulator->getRobotAtPosition(point_in_scene);
    }
    else if (r_clicked && standalone_simulator)
    {
        initial_click_point = createPoint(mapToScene(event->pos()));
        robot = standalone_simulator->getRobotAtPosition(initial_click_point);
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
        final_click_point = createPoint(mapToScene(event->pos()));
        standalone_simulator->setBallState(
            BallState(initial_click_point, initial_click_point - final_click_point));
    }

    resetFlags();

    DrawFunctionVisualizer::mouseReleaseEvent(event);
}

void StandaloneSimulatorDrawFunctionVisualizer::mouseMoveEvent(QMouseEvent* event)
{
    auto physics_robot = robot.lock();
    if (physics_robot && standalone_simulator)
    {
        if (shift_clicked)
        {
            Point point_in_scene = createPoint(mapToScene(event->pos()));
            physics_robot->setPositionAndOrientation(point_in_scene,
                                                     physics_robot->orientation());
        }
        else if (r_clicked)
        {
            final_click_point = createPoint(mapToScene(event->pos()));
            Vector vec        = Vector(final_click_point.x() - initial_click_point.x(),
                                final_click_point.y() - initial_click_point.y());
            Angle angle       = vec.orientation();
            physics_robot->setPositionAndOrientation(physics_robot->position(), angle);
        }
    }
    else if (ctrl_clicked && standalone_simulator)
    {
        final_click_point = createPoint(mapToScene(event->pos()));
    }

    DrawFunctionVisualizer::mouseMoveEvent(event);
}

WorldDrawFunction StandaloneSimulatorDrawFunctionVisualizer::getDrawBallVelocityFunction()
{
    if (ctrl_clicked)
    {
        // Draw function will draw a line if the user is adding velocity to ball
        auto draw_function = [this](QGraphicsScene* scene) {
            drawBallVelocity(scene, initial_click_point,
                             initial_click_point - final_click_point,
                             ball_speed_slow_color, ball_speed_fast_color);
        };
        return WorldDrawFunction(draw_function);
    }
    else
    {
        // Draw function draws a vector of magnitude 0 if user is not manipulating the
        // ball This is necessary to prevent a line being drawn on screen when
        // manipulating the robots.
        auto draw_function = [this](QGraphicsScene* scene) {
            drawBallVelocity(scene, initial_click_point, Vector(0, 0),
                             ball_speed_slow_color, ball_speed_fast_color);
        };
        return WorldDrawFunction(draw_function);
    }
}

void StandaloneSimulatorDrawFunctionVisualizer::contextMenuEvent(QContextMenuEvent* event)
{
    Point point_in_scene    = createPoint(mapToScene(event->pos()));
    auto robot_under_cursor = standalone_simulator->getRobotAtPosition(point_in_scene);

    QMenu menu(this);
    menu.addAction("Reset View",
                   [this]() { DrawFunctionVisualizer::setViewArea(last_view_area); });
    menu.addAction("Place Ball Here", [&]() {
        standalone_simulator->setBallState(BallState(point_in_scene, Vector(0, 0), 0));
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
