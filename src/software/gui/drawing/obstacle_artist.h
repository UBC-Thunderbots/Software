#pragma once

#include <QtWidgets/QGraphicsScene>

#include "proto/geometry.pb.h"
#include "proto/visualization.pb.h"
#include "software/ai/navigator/obstacle/obstacle.hpp"
#include "software/ai/navigator/obstacle/obstacle_visitor.h"
#include "software/gui/drawing/colors.h"
#include "software/gui/drawing/draw_functions.h"
#include "software/gui/drawing/geom.h"
#include "software/gui/geometry_conversion.h"

// DO NOT INCLUDE IN PR

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
    explicit ObstacleArtist(QGraphicsScene* scene, std::optional<QPen> pen);

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
    void visit(const GeomObstacle<Rectangle>& geom_obstacle) override;

    /**
     * Log the obstacle protobuf to the VISUALIZE log level and clear it
     */
    void visualize();

   private:
    QGraphicsScene* scene_;
    std::optional<QPen> pen_;
    TbotsProto::Obstacles obstacle_proto_;
};