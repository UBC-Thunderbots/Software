#include "software/gui/generic_widgets/world_view/draw_function_visualizer.h"
#include "software/gui/geometry_conversion.h"
#include "software/logger/logger.h"

DrawFunctionVisualizer::DrawFunctionVisualizer(QWidget *parent) : ZoomableQGraphicsView(parent), graphics_scene(new QGraphicsScene(this)), open_gl_widget(new QOpenGLWidget(this))
{
    setScene(graphics_scene);
    setDragMode(QGraphicsView::ScrollHandDrag);
    setBackgroundBrush(QBrush(Qt::darkGreen, Qt::SolidPattern));

    // Performance optimizations
    // https://stackoverflow.com/questions/43826317/how-to-optimize-qgraphicsviews-performance
    setInteractive(false);
    setCacheMode(QGraphicsView::CacheBackground);
    setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
    // Using an OpenGL widget with the view should help make use of the graphics card
    // rather than doing CPU drawing, which should take some load off the CPU and make
    // things faster
    setViewport(open_gl_widget);

    setRenderHint(QPainter::Antialiasing);

    // Invert the y-coordinates of the view.
    // We do this because Qt's default coordinate system for drawing is:
    // * positive x = "right"
    // * positive y = "down"
    // Our coordinate system defines positive y as being "up", so we invert the coordinate
    // system so all our draw calls can follow our coordinate convention. This also fixes
    // the orientation convention, so "positive" rotation is counterclockwise in the view
    // (rotating from +x, to +y, to -x, to -y)
    QTransform view_transform(1, 0, 0, -1, 0, 0);
    setTransform(view_transform);

    update();
}

void DrawFunctionVisualizer::clearAndDraw(const std::vector<DrawFunction> &draw_functions) {
    graphics_scene->clear();
    for (auto draw_function : draw_functions)
    {
        if(draw_function) {
            draw_function(graphics_scene);
        }else {
            LOG(WARNING) << "Attempted to draw a non-callable DrawFunction";
        }
    }
}

void DrawFunctionVisualizer::setViewArea(const Rectangle &view_area) {
    graphics_scene->setSceneRect(createQRectF(view_area));
    fitInView(graphics_scene->sceneRect(), Qt::KeepAspectRatio);
}
