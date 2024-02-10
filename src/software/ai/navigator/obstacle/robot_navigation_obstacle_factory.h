#pragma once

#include "proto/parameters.pb.h"
#include "proto/primitive.pb.h"
#include "software/ai/navigator/obstacle/obstacle.hpp"
#include "software/ai/navigator/trajectory/trajectory_path.h"
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
     * Create obstacles for the given motion constraints
     *
     * @param motion_constraints The motion constraints to create obstacles for
     * @param field Field we're enforcing motion constraints in
     *
     * @return Obstacles representing the given motion constraints
     */
    std::vector<ObstaclePtr> createObstaclesFromMotionConstraints(
        const std::set<TbotsProto::MotionConstraint> &motion_constraints,
        const World &world) const;

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
     * @param motion_constraints The motion constraints to create obstacles for
     * @param field Field we're enforcing motion constraints in
     *
     * @return Obstacles representing the given motion constraints
     */
    std::vector<ObstaclePtr> createDynamicObstaclesFromMotionConstraints(
        const std::set<TbotsProto::MotionConstraint> &motion_constraints,
        const World &world) const;

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
     * Create a stadium shaped obstacle for enemy robot with additional radius scaling
     *
     * @param enemy_robot Enemy robot to create the obstacle for
     *
     * @return obstacle around the robot
     */
    ObstaclePtr createStadiumEnemyRobotObstacle(const Robot& enemy_robot) const;


    /**
     * Create dynamic circle obstacle for enemy robot with additional radius scaling
     *
     * @param enemy_robot Enemy robot to create the obstacle for
     *
     * @return obstacle around the robot
     */
    ObstaclePtr createConstVelocityEnemyRobotObstacle(const Robot& enemy_robot) const;

    /**
     * Create dynamic circle obstacle around robot with additional radius scaling
     *
     * @param robot robot to create the obstacle for
     * @param traj Trajectory which the obstacle is following
     *
     * @return moving obstacle around the robot
     */
     ObstaclePtr createFromMovingRobot( // TODO (NIMA): test
     const Robot &robot, const TrajectoryPath& traj) const;

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
    ObstaclePtr createFromShape(const Rectangle &rectangle) const;
    ObstaclePtr createFromShape(const Stadium &stadium) const;

    /**
     * Generate a trajectory based circular obstacle with additional radius scaling
     *
     * @param circle The circle to make obstacle with
     * @param traj Trajectory which the obstacle is following
     */
    ObstaclePtr createCircleWithTrajectory(const Circle &circle, const TrajectoryPath& traj) const;

    /**
     * Generate a const velocity based circular obstacle with additional radius scaling
     *
     * @param circle The circle to make obstacle with
     * @param velocity The velocity of the obstacle
     */
    ObstaclePtr createCircleWithConstVelocity(const Circle &circle, const Vector& velocity) const;

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
