#pragma once

#include <QtWidgets/QWidget>

#include "software/gui/generic_widgets/draw_function_visualizer/draw_function_visualizer.h"
#include "software/world/robot_state.h"
#include "software/world/team_colour.h"
#include "software/new_geom/point.h"
#include "software/simulation/physics/physics_robot.h"
#include <memory>

/**
 * A custom version of the DrawFunctionVisualizer widget that allows the user to
 * interact with the simulation, such as clicking to place the ball
 */
class StandaloneSimulatorDrawFunctionVisualizer : public DrawFunctionVisualizer
{
    Q_OBJECT

   public:
    explicit StandaloneSimulatorDrawFunctionVisualizer(QWidget* parent = 0);

    /**
     * Register a callback that will be called when the user requests to place the ball
     * in the GUI.
     *
     * @param callback The callback to register
     */
    void registerBallPlacementCallback(const std::function<void(Point)>& callback);

    /**
     * Sets the function used to get a robot at the given position. This is used to
     * determine what robot to move when the user is placing robots.
     *
     * @param func The new function to set
     */
    void setGetRobotAtPositionFunc(const std::function<std::weak_ptr<PhysicsRobot>(Point)>& func);

   private:
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;

    std::vector<std::function<void(Point)>> ball_placement_callbacks;
    std::function<std::weak_ptr<PhysicsRobot>(Point)> get_robot_at_position_func;
    std::weak_ptr<PhysicsRobot> robot;
};
