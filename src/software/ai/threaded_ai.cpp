#include "software/ai/threaded_ai.h"

#include <boost/bind.hpp>

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/gui/drawing/navigator.h"

ThreadedAI::ThreadedAI(std::shared_ptr<const AiConfig> ai_config,
                       std::shared_ptr<const AiControlConfig> control_config,
                       std::shared_ptr<const PlayConfig> play_config)
    // Disabling warnings on log buffer full, since buffer size is 1 and we always want AI
    // to use the latest World
    : FirstInFirstOutThreadedObserver<World>(DEFAULT_BUFFER_SIZE, false),
      ai(ai_config, control_config, play_config),
      control_config(control_config)
{
}

void ThreadedAI::onValueReceived(World world)
{
    runAIAndSendPrimitives(world);
    drawAI();
}

void ThreadedAI::runAIAndSendPrimitives(const World& world)
{
    if (control_config->getRunAi()->value())
    {
        auto new_primitives = ai.getPrimitives(world);

        PlayInfo play_info = ai.getPlayInfo();
        Subject<PlayInfo>::sendValueToObservers(play_info);

        Subject<TbotsProto::PrimitiveSet>::sendValueToObservers(*new_primitives);
    }
}

void ThreadedAI::drawAI()
{
    if (ai.getNavigator())
    {
        // TODO HACK (#2167) if we don't combine these draw functions
        // the visualizer flickers waaay too much. We need better and
        // more generic layering support
        auto planned_paths      = ai.getNavigator()->getPlannedPathPoints();
        auto obstacles          = ai.getNavigator()->getObstacles();
        auto circles_with_color = ai.getCirclesWithColorToDraw();
        auto draw_function      = [circles_with_color, planned_paths,
                              obstacles](QGraphicsScene* scene) {
            QPen path_pen(navigator_path_color);
            // The cap style must be NOT be set to SquareCap. It can be set to anything
            // else. Drawing a line of length 0 with the SquareCap style causes a large
            // line to be drawn
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
            // The cap style must be NOT be set to SquareCap. It can be set to anything
            // else. Drawing a line of length 0 with the SquareCap style causes a large
            // line to be drawn
            obstacle_pen.setCapStyle(Qt::PenCapStyle::RoundCap);
            obstacle_pen.setWidth(2);
            obstacle_pen.setCosmetic(true);

            ObstacleArtist obstacle_artist(scene, obstacle_pen);
            for (const auto& obstacle : obstacles)
            {
                obstacle->accept(obstacle_artist);
            }
            for (auto circle_with_color : circles_with_color)
            {
                // TODO (#2167) put this somwehere else
                if (circle_with_color.second == "pink")
                {
                    QPen pen(pink);
                    pen.setWidth(2);
                    pen.setCosmetic(true);
                    drawCircle(scene, circle_with_color.first, pen);
                }
                if (circle_with_color.second == "blue")
                {
                    QPen pen(blue);
                    pen.setWidth(2);
                    pen.setCosmetic(true);
                    drawCircle(scene, circle_with_color.first, pen);
                }
            }
        };
        Subject<AIDrawFunction>::sendValueToObservers(AIDrawFunction(draw_function));
    }
}
