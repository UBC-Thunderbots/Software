#include "software/gui/drawing/navigator.h"

AIDrawFunction drawNavigator(std::shared_ptr<Navigator> navigator)
{
    auto planned_paths = navigator->getPlannedPathPoints();
    auto obstacles     = navigator->getObstacles();
    auto draw_function = [planned_paths, obstacles](QGraphicsScene* scene) {
        QPen path_pen(navigator_path_color);
        // The cap style must be NOT be set to SquareCap. It can be set to anything else.
        // Drawing a line of length 0 with the SquareCap style causes a large line to be
        // drawn
        path_pen.setCapStyle(Qt::PenCapStyle::RoundCap);
        path_pen.setWidth(2);
        path_pen.setCosmetic(true);
        path_pen.setStyle(Qt::DashLine);
        // Create a set a custom dash pattern. We do this because the default
        // patterns don't have enough space between the dashes so aren't easily
        // distinguishable as dashed lines.
        QVector<qreal> dashes;
        qreal space = 7;
        dashes << 2 << space << 2 << space;
        path_pen.setDashPattern(dashes);

        for (const auto& path : planned_paths)
        {
            for (size_t i = 1; i < path.size(); i++)
            {
                Segment path_segment(path[i - 1], path[i]);
                drawSegment(scene, path_segment, path_pen);
            }
        }

        QPen obstacle_pen(navigator_obstacle_color);
        // The cap style must be NOT be set to SquareCap. It can be set to anything else.
        // Drawing a line of length 0 with the SquareCap style causes a large line to be
        // drawn
        obstacle_pen.setCapStyle(Qt::PenCapStyle::RoundCap);
        obstacle_pen.setWidth(2);
        obstacle_pen.setCosmetic(true);

        ObstacleArtist obstacle_artist(scene, std::make_optional<QPen>(obstacle_pen));
        for (const auto& obstacle : obstacles)
        {
            obstacle->accept(obstacle_artist);
        }
        obstacle_artist.visualize();
    };

    return AIDrawFunction(draw_function);
}
