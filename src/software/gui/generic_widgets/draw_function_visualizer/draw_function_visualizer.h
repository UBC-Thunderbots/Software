#pragma once

#include <QtWidgets/QGraphicsScene>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QOpenGLWidget>
#include "software/gui/generic_widgets/draw_function_visualizer/zoomable_qgraphics_view.h"
#include "software/gui/drawing/draw_functions.h"
#include "software/new_geom/rectangle.h"

// TODO: protected inheritance to limit interface?
class DrawFunctionVisualizer : public ZoomableQGraphicsView {
    Q_OBJECT

public:
    explicit DrawFunctionVisualizer(QWidget* parent = 0);

    void clearAndDraw(const std::vector<DrawFunction>& draw_functions);

    /**
     * Sets the area of the scene that's visible in the view
     *
     * @param view_area The new area to show in the view
     */
    void setViewArea(const Rectangle& view_area);

private:
    // The "parent" of each of these widgets is set during construction, meaning that
    // the Qt system takes ownership of the pointer and is responsible for de-allocating
    // it, so we don't have to
    QGraphicsScene* graphics_scene;
    QOpenGLWidget* open_gl_widget;
};


/**
 * Sets up the SceneView which draws the AI's view of the world, such as robot
 * positions, ball velocity, etc.
 *
 * @param view The view to display the QGraphicsScene with
 * @param scene The QGraphicsScene that will be displayed by the view
 * @param gl_widget A QOpenGLWidget that can be used to help display items in the view
 */
//void setupSceneView(QGraphicsView* view, QGraphicsScene* scene, QOpenGLWidget* gl_widget);
