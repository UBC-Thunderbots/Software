#include "software/visualizer/drawing/navigator.h"

#include "software/util/variant_visitor/variant_visitor.h"

AIDrawFunction drawNavigator(std::shared_ptr<Navigator> navigator)
{
    auto planned_paths = navigator->getPlannedPathPoints();
    auto obstacles     = navigator->getObstacles();
    auto draw_function = [planned_paths, obstacles](QGraphicsScene* scene) {
        QPen pen(navigator_path_color);
        // The cap style must be NOT be set to SquareCap. It can be set to anything else.
        // Drawing a line of length 0 with the SquareCap style causes a large line to be
        // drawn
        pen.setCapStyle(Qt::PenCapStyle::RoundCap);
        pen.setWidth(2);
        pen.setCosmetic(true);

        for (const auto& path : planned_paths)
        {
            for (size_t i = 1; i < path.size(); i++)
            {
                Segment path_segment(path[i - 1], path[i]);
                drawSegment(scene, path_segment, pen);
            }
        }

        //        for (const auto& obstacle : obstacles)
        //        {
        //            drawObstacle(scene, obstacle, pen);
        //        }
    };

    return AIDrawFunction(draw_function);
}

void drawObstacle(QGraphicsScene* scene, const ObstaclePtr& obstacle_ptr, const QPen& pen)
{
    //    std::visit(
    //        overload{
    //            [scene, pen](const Circle& circle) { drawCircle(scene, circle, pen); },
    //            [scene, pen](const Polygon& polygon) { drawPolygon(scene, polygon, pen);
    //            }},
    //        obstacle_shape);
}
