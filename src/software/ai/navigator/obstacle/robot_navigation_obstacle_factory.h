#pragma once

#include "proto/parameters.pb.h"
#include "proto/primitive.pb.h"
#include "software/ai/navigator/obstacle/obstacle.hpp"
#include "software/geom/line.h"
#include "software/geom/point.h"
#include "software/geom/polygon.h"
#include "software/logger/logger.h"
#include "software/world/world.h"

/**
 * The RobotNavigationObstacleFactory creates obstacles for navigation with a robot
 * NOTE: All obstacles created include at least an additional robot radius margin on all
 * sides of the obstacle
 */
class RobotNavigationObstacleFactory
{
   public:
    /**
     * Create an RobotNavigationObstacleFactory with the given configuration
     *
     * @param config The configuration used to determine how obstacles should be generated
     */
    RobotNavigationObstacleFactory(TbotsProto::RobotNavigationObstacleConfig config);

    /**
     * Create static or dynamic obstacles for the given motion constraint
     *
     * @param motion_constraint The motion constraint to create obstacle for
     * @param world World we're enforcing motion constraints in
     *
     * @return Obstacles representing the given motion constraint
     */
    std::vector<ObstaclePtr> createFromMotionConstraint(
        const TbotsProto::MotionConstraint motion_constraint, const World &world) const;

    /**
     * Create static obstacles for the given motion constraints
     *
     * @param motion_constraints The motion constraints to create obstacles for
     * @param field Field we're enforcing motion constraints in
     *
     * @return Obstacles representing the given motion constraints
     */
    std::vector<ObstaclePtr> createStaticObstaclesFromMotionConstraints(
        const std::set<TbotsProto::MotionConstraint> &motion_constraints,
        const Field &field) const;

    /**
     * Create dynamic obstacles for the given motion constraints
     *
     * @param motion_constraint The motion constraint to create obstacles for
     * @param world World we're enforcing motion constraints in
     *
     * @return Obstacles representing the given motion constraint
     */
    std::vector<ObstaclePtr> createDynamicObstaclesFromMotionConstraint(
        const TbotsProto::MotionConstraint &motion_constraint, const World &world) const;

    /**
     * Create static obstacles for the given motion constraint
     *
     * @param motion_constraint The motion constraint to create obstacles for
     * @param field Field we're enforcing motion constraints in
     *
     * @return Obstacles representing the given motion constraint
     */
    std::vector<ObstaclePtr> createStaticObstaclesFromMotionConstraint(
        const TbotsProto::MotionConstraint &motion_constraint, const Field &field) const;

    /**
     * Create circle obstacle around robot with additional radius scaling
     *
     * @param robot_position robot position
     *
     * @return obstacle around the robot
     */
    ObstaclePtr createFromRobotPosition(const Point &robot_position) const;

    /**
     * Create circle obstacle around ball
     *
     * @param ball_position ball position to make obstacle around
     *
     * @return obstacle around the ball
     */
    ObstaclePtr createFromBallPosition(const Point &ball_position) const;

    /**
     * Returns an obstacle for the shape
     * NOTE: as with all other obstacles created by RobotNavigationObstacleFactory, the
     * shapes are expanded on all sides to account for the radius of the robot
     *
     * @param The shape to expand
     *
     * @return ObstaclePtr
     */
    ObstaclePtr createFromShape(const Circle &circle) const;
    ObstaclePtr createFromShape(const Polygon &polygon) const;

    /**
     * Returns an obstacle with the shape of the BallPlacementZone if the state is in
     * enemy ball placement
     *
     * @param placement_point the point where the ball will be placed
     * @param ball_point the point where the ball currently is
     *
     * @return ObstaclePtr
     */
    ObstaclePtr createFromBallPlacement(const Point &placement_point,
                                        const Point &ball_point) const;

   private:
    TbotsProto::RobotNavigationObstacleConfig config;
    double robot_radius_expansion_amount;

    /**
     * Returns an obstacle for the field_rectangle expanded on all sides to account for
     * the size of the robot. If a side of the field_rectangle lies along a field line,
     * then it is expanded out to the field boundary
     *
     * @param field_rectangle The rectangle to make obstacle
     * @param field_lines The rectangle representing field lines
     * @param field_boundary The rectangle representing field boundary
     * @param additional_expansion_amount (optional) The amount to expand all sides of the
     * rectangle in addition to the robot radius
     *
     * @return ObstaclePtr
     */
    ObstaclePtr createFromFieldRectangle(const Rectangle &field_rectangle,
                                         const Rectangle &field_lines,
                                         const Rectangle &field_boundary,
                                         double additional_expansion_amount = 0.0) const;
};
