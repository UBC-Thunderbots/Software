#include "software/gui/generic_widgets/world_view/world_view.h"

void setupSceneView(QGraphicsView* view, QGraphicsScene* scene, QOpenGLWidget* gl_widget)
{
    view->setScene(scene);
    view->setDragMode(QGraphicsView::ScrollHandDrag);
    view->setBackgroundBrush(QBrush(Qt::darkGreen, Qt::SolidPattern));

    // Performance optimizations
    // https://stackoverflow.com/questions/43826317/how-to-optimize-qgraphicsviews-performance
    view->setInteractive(false);
    view->setOptimizationFlag(QGraphicsView::DontAdjustForAntialiasing);
    view->setCacheMode(QGraphicsView::CacheBackground);
    view->setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
    // Using an OpenGL widget with the view should help make use of the graphics card
    // rather than doing CPU drawing, which should take some load off the CPU and make
    // things faster
    view->setViewport(gl_widget);

    // Invert the y-coordinates of the view.
    // We do this because Qt's default coordinate system for drawing is:
    // * positive x = "right"
    // * positive y = "down"
    // Our coordinate system defines positive y as being "up", so we invert the coordinate
    // system so all our draw calls can follow our coordinate convention. This also fixes
    // the orientation convention, so "positive" rotation is counterclockwise in the view
    // (rotating from +x, to +y, to -x, to -y)
    QTransform view_transform(1, 0, 0, -1, 0, 0);
    view->setTransform(view_transform);

    view->update();
}
