#include "software/gui/standalone_simulator/widgets/standalone_simulator_draw_function_visualizer.h"

#include "software/gui/geometry_conversion.h"

StandaloneSimulatorDrawFunctionVisualizer::StandaloneSimulatorDrawFunctionVisualizer(
    QWidget* parent)
    : DrawFunctionVisualizer(parent)
{
}

void StandaloneSimulatorDrawFunctionVisualizer::registerBallPlacementCallback(
    const std::function<void(Point)>& callback)
{
    ball_placement_callbacks.emplace_back(callback);
}

void StandaloneSimulatorDrawFunctionVisualizer::mousePressEvent(QMouseEvent* event)
{
    // If Ctrl is pressed, place the ball where the user clicks
    if (event->modifiers() & Qt::ControlModifier)
    {
        Point point_in_scene = createPoint(mapToScene(event->pos()));
        for (const auto& callback : ball_placement_callbacks)
        {
            callback(point_in_scene);
        }
    }
    else
    {
        DrawFunctionVisualizer::mousePressEvent(event);
    }
}
