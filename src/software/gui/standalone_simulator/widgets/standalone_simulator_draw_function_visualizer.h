#pragma once

#include <QtWidgets/QWidget>

#include "software/gui/generic_widgets/draw_function_visualizer/draw_function_visualizer.h"

class StandaloneSimulatorDrawFunctionVisualizer : public DrawFunctionVisualizer {
    Q_OBJECT

public:
    StandaloneSimulatorDrawFunctionVisualizer(QWidget* parent = 0);
    void registerBallPlacementCallback(const std::function<void(Point)>& callback);

private:
    void mousePressEvent(QMouseEvent* event) override;

    std::vector<std::function<void(Point)>> ball_placement_callbacks;
};