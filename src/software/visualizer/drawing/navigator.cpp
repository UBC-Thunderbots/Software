#include "software/visualizer/drawing/navigator.h"

#include <QtWidgets/QGraphicsScene>

#include "software/geom/segment.h"
#include "software/visualizer/drawing/colors.h"
#include "software/visualizer/geom/geometry_conversion.h"

AIDrawFunction drawNavigator(std::shared_ptr<Navigator> navigator)
{
    auto planned_paths = navigator->getPlannedPathPoints();
    auto draw_function = [planned_paths](QGraphicsScene* scene) {
        QPen pen(navigator_path_color);
        // The cap style must be NOT be set to SquareCap. It can be set to anything else.
        // Drawing a line of length 0 with the SquareCap style causes a large line to be
        // drawn
        pen.setCapStyle(Qt::PenCapStyle::RoundCap);
        pen.setWidth(2);
        pen.setCosmetic(true);

        QBrush brush(navigator_path_color);
        brush.setStyle(Qt::BrushStyle::SolidPattern);

        for (const auto& path : planned_paths)
        {
            for (size_t i = 1; i < path.size(); i++)
            {
                Segment path_segment(path[i - 1], path[i]);
                QLineF line = createQLineF(path_segment);
                scene->addLine(line, pen);
            }
        }
    };

    return AIDrawFunction(draw_function);
}
