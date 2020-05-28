#pragma once

#include "software/ai/motion_constraint/motion_constraint.h"
#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/geom/util.h"
#include "software/logger/logger.h"
#include "software/new_geom/point.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/world/world.h"

/**
 * The obstacle factory creates obstacles with an additional robot radius for navigation
 */
class ObstacleFactory
{
   public:
    ObstacleFactory() = delete;

    /**
     * Create an ObstacleFactory with the given configuration
     *
     * @param config The configuration used to determine how obstacles should be generated
     */
    ObstacleFactory(std::shared_ptr<const ObstacleFactoryConfig> config);

    /**
     * Create obstacles for the given motion constraints
     *
     * @param motion_constraints The motion constraints to create obstacles for
     * @param world World we're enforcing motion constraints in
     *
     * @return Obstacles representing the given motion constraints
     */
    std::vector<ObstaclePtr> createObstaclesFromMotionConstraints(
        const std::set<MotionConstraint> &motion_constraints, const World &world) const;

    /**
     * Create obstacles for the given motion constraint
     *
     * @param motion_constraint The motion constraint to create obstacles for
     * @param world World we're enforcing motion constraints in
     *
     * @return Obstacles representing the given motion constraint
     */
    std::vector<ObstaclePtr> createObstaclesFromMotionConstraint(
        const MotionConstraint &motion_constraint, const World &world) const;

    /**
     * Create an obstacle representing the given robot
     *
     * These obstacles take into account the velocity of the robot to extend the
     * created obstacle in the robot's direction of travel.
     *
     * @param robot The robot to get a representative obstacle for
     *
     * @return An obstacle representing the given robot
     */
    ObstaclePtr createVelocityObstacleFromRobot(const Robot &robot) const;

    /**
     * Create a list of obstacles representing the given team
     *
     * These obstacles take into account the velocity of the robot to extend the
     * created obstacle in the robot's direction of travel.
     *
     * @param team The team to get representative obstacles for
     *
     * @return A list of obstacles representing the given team
     */
    std::vector<ObstaclePtr> createVelocityObstaclesFromTeam(const Team &team) const;

    /**
     * Create circle obstacle around robot with additional radius scaling
     *
     * @param robot_position robot position
     *
     * @return obstacle around the robot
     */
    ObstaclePtr createRobotObstacle(const Point &robot_position) const;

    /**
     * Create circle obstacle around ball
     *
     * @param ball_position ball position to make obstacle around
     *
     * @return obstacle around the ball
     */
    ObstaclePtr createBallObstacle(const Point &ball_position) const;

    /**
     * Create rectangle-shaped obstacle
     *
     * @param rectangle Rectangle to make obstacle with
     *
     * @return rectangular obstacle
     */
    ObstaclePtr createObstacleFromRectangle(const Rectangle &rectangle) const;

   private:
    std::shared_ptr<const ObstacleFactoryConfig> config;
    double obstacle_expansion_amount;

    /**
     * Returns an obstacle for the shapep expanded on all sides to account for the size of
     * the robot
     *
     * @param The shape to expand
     *
     * @return expanded ObstaclePtr
     */
    ObstaclePtr expandForRobotSize(const Circle &circle) const;
    ObstaclePtr expandForRobotSize(const Polygon &polygon) const;


    /**
     * Returns an obstacle from the Rectangle expanded on +x (if friendly), -x (if enemy),
     * +y, and -y sides to account for the size of the robot
     *
     * @param rectangle Rectangle to expand
     * @param team_type The side that the Rectangle belongs to
     *
     * @return expanded Rectangle as an obstacle
     */
    ObstaclePtr expandThreeSidesForRobotSize(const Rectangle &rectangle,
                                             TeamType team_type) const;

    /**
     * Returns an obstacle from the Rectangle expanded on +x (if friendly), -x (if enemy)
     * sides to account for the size of the robot
     *
     * @param rectangle Rectangle to expand
     * @param team_type The side that the Rectangle belongs to
     *
     * @return expanded Rectangle as an obstacle
     */
    ObstaclePtr expandOneSideForRobotSize(const Rectangle &rectangle,
                                          TeamType team_type) const;

    /**
     * Returns an obstacle from the Rectangle expanded on +x (if friendly), -x (if enemy),
     * +y, and -y sides to account for the size of the robot plus an additional inflation
     * amount
     *
     * @param rectangle Rectangle to expand
     * @param team_type The side that the Rectangle belongs to
     *
     * @return expanded Rectangle as an obstacle
     */
    ObstaclePtr expandThreeSidesForInflatedRobotSize(const Rectangle &rectangle,
                                                     TeamType team_type) const;
};
