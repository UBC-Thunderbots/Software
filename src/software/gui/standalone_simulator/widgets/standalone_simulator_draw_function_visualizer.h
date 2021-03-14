#pragma once

#include <QtWidgets/QWidget>
#include <memory>

#include "software/geom/point.h"
#include "software/gui/generic_widgets/draw_function_visualizer/draw_function_visualizer.h"
#include "software/simulation/physics/physics_robot.h"
#include "software/simulation/standalone_simulator.h"
#include "software/world/robot_state.h"
#include "software/world/team_types.h"

/**
 * A custom version of the DrawFunctionVisualizer widget that allows the user to
 * interact with the simulation, such as clicking to place the ball
 */
class StandaloneSimulatorDrawFunctionVisualizer : public DrawFunctionVisualizer
{
    Q_OBJECT

   public:
    explicit StandaloneSimulatorDrawFunctionVisualizer(QWidget* parent = nullptr);

    /**
     * Sets the StandaloneSimulator that this widget controls
     *
     * @param simulator The standalone simulator to control
     */
    void setStandaloneSimulator(std::shared_ptr<StandaloneSimulator> simulator);


    /**
     * Gets the WorldDrawFunction for the Simulator to draw the
     * vector when the ball is moving
     *
     */
    WorldDrawFunction getDrawBallVelocityFunction();

   private:
    void keyPressEvent(QKeyEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void contextMenuEvent(QContextMenuEvent* event) override;

    // The robot currently being placed by the user, if any
    std::weak_ptr<PhysicsRobot> robot;

    // Boolean to check if user shift clicked
    bool shift_clicked = false;

    // Boolean to check if the user ctrl clicked
    bool ctrl_clicked = false;

    // Boolean to check if user has r pressed
    bool r_clicked = false;

    // The final point when adding force to the ball, or rotating robot, if any
    Point initial_point;

    // The final point when adding force to the ball, or rotating robot, if any
    Point final_point;

    std::shared_ptr<StandaloneSimulator> standalone_simulator;
};
