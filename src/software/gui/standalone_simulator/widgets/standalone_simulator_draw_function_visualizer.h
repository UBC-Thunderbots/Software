#pragma once

#include <QtWidgets/QWidget>

#include "software/gui/generic_widgets/draw_function_visualizer/draw_function_visualizer.h"

/**
 * A custom version of the DrawFunctionVisualizer widget that allows the user to
 * interact with the simulation, such as clicking to place the ball
 */
class StandaloneSimulatorDrawFunctionVisualizer : public DrawFunctionVisualizer
{
    Q_OBJECT

   public:
    StandaloneSimulatorDrawFunctionVisualizer(QWidget* parent = 0);

    /**
     * Register a callback that will be called when the user requests to place the ball
     * in the GUI.
     *
     * @param callback The callback to register
     */
    void registerBallPlacementCallback(const std::function<void(Point)>& callback);

   private:
    void mousePressEvent(QMouseEvent* event) override;

    std::vector<std::function<void(Point)>> ball_placement_callbacks;
};