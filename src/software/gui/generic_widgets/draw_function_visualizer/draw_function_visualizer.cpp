#include "software/gui/generic_widgets/draw_function_visualizer/draw_function_visualizer.h"

#include <QtWidgets/QMenu>
#include <string>

#include "software/gui/drawing/colors.h"
#include "software/gui/geometry_conversion.h"
#include "software/logger/logger.h"

DrawFunctionVisualizer::DrawFunctionVisualizer(QWidget *parent)
    : ZoomableQGraphicsView(parent),
      graphics_scene(new QGraphicsScene(this)),
      open_gl_widget(new QOpenGLWidget(this)),
      // Placeholder Rectangle
      last_view_area(Rectangle(Point(1, 1), Point(0, 0)))

{
    setScene(graphics_scene);

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
    setDragMode(QGraphicsView::ScrollHandDrag);
    setBackgroundBrush(QBrush(field_color, Qt::SolidPattern));
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

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

    // The SceneRect defines the bounding rectangle of the scene, outside of which
    // items cannot exist. The graphics_scene SceneRect also defines the scrollable
    // area for the QGraphicsView. We want to give the user the freedom to scroll
    // around the scene as much as they please, so we set this rectangle to be
    // very large
    //    Point bottom_left(std::numeric_limits<float>::lowest(),
    //    std::numeric_limits<float>::lowest()); Point
    //    top_right(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    //    Rectangle max_scene_rect(bottom_left, top_right);
    //    graphics_scene->setSceneRect(createQRectF(max_scene_rect));

    update();
}

void DrawFunctionVisualizer::clearAndDraw(const std::vector<DrawFunction> &draw_functions)
{
    graphics_scene->clear();
    for (auto draw_function : draw_functions)
    {
        if (draw_function)
        {
            draw_function(graphics_scene);
        }
        else
        {
            LOG(WARNING) << "Attempted to draw a non-callable DrawFunction";
        }
    }
}

void DrawFunctionVisualizer::setViewArea(const Rectangle &view_area)
{
    // Moves and scales the view to fit the view_area in the scene
    last_view_area = view_area;
    fitInView(createQRectF(view_area), Qt::KeepAspectRatio);
}

void DrawFunctionVisualizer::contextMenuEvent(QContextMenuEvent *event)
{
    QMenu menu(this);
    menu.addAction("Reset View",
                   [this]() { DrawFunctionVisualizer::setViewArea(last_view_area); });

    menu.exec(event->globalPos());
}
