#pragma once

#include <QtWidgets/QGraphicsScene>
#include <QtWidgets/QOpenGLWidget>

#include "software/geom/rectangle.h"
#include "software/gui/drawing/draw_functions.h"
#include "software/gui/generic_widgets/draw_function_visualizer/zoomable_qgraphics_view.h"

/**
 * This class is a QGraphicsView widget that allows the user to zoom
 * and pan around the scene. It provides an interface for drawing arbitrary
 * shapes and information in the scene through the use of DrawFunctions.
 */
class DrawFunctionVisualizer : public ZoomableQGraphicsView
{
    Q_OBJECT

   public:
    explicit DrawFunctionVisualizer(QWidget* parent = nullptr);

    /**
     * Clears the scene and draws each of the provided DrawFunctions in order
     *
     * @param draw_functions The DrawFunctions to draw on the scene, in order
     */
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

   protected:
    // Functions to be inherited by standalone_simulator_draw_function_visualizer

    /**
     * When the context menu is called by right-clicking on the simulator field
     *
     * @param event The QContextMenuEvent to open
     */
    void contextMenuEvent(QContextMenuEvent* event) override;

    /**
     * Called to get the reset view function reference
     *
     * @return a reference of a function to reset view of the simulator
     */
    std::function<void()> createResetView();

    /**
     * Resets the view of the simulator, using the last saved {@code Rectangle
     * lastViewArea} as reference
     */
    void resetView(const Rectangle& view_area);

    int remaining_attempts_to_set_view_area;
    static constexpr int NUM_ATTEMPTS_TO_SET_INITIAL_VIEW_AREA = 5;

    Rectangle lastViewArea;
};
