#pragma once

#include <QtWidgets/QGraphicsScene>

#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/ai/navigator/obstacle/obstacle_visitor.h"
#include "software/gui/visualizer/drawing/colors.h"
#include "software/gui/visualizer/drawing/draw_functions.h"
#include "software/gui/visualizer/drawing/geom.h"
#include "software/gui/visualizer/geom/geometry_conversion.h"

/**
 * The ObstacleArtist draws Obstacles with artistic flair
 */
class ObstacleArtist : public ObstacleVisitor
{
   public:
    /**
     * Create an ObstacleArtist to draw on the given scene with the given pen
     *
     * @param scene The scene to draw on
     * @param pen The QPen to draw the obstacle
     */
    explicit ObstacleArtist(QGraphicsScene* scene, const QPen& pen);

    /**
     * Draws the ObstaclePtr on the given scene.
     *
     * @param obstacle_ptr The ObstaclePtr to draw
     */
    void drawObstacle(const ObstaclePtr& obstacle_ptr);

    /**
     * Draws the given Obstacle
     *
     * @param The Obstacle to draw
     */
    void visit(const GeomObstacle<Circle>& geom_obstacle) override;
    void visit(const GeomObstacle<Polygon>& geom_obstacle) override;

   private:
    QGraphicsScene* scene_;
    QPen pen_;
};
