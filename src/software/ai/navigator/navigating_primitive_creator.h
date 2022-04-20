#pragma once

#include "proto/parameters.pb.h"
#include "proto/primitive.pb.h"
#include "software/ai/intent/all_intents.h"
#include "software/ai/intent/navigating_intent.h"
#include "software/ai/navigator/obstacle/obstacle.hpp"
#include "software/ai/navigator/path_planner/path_planner.h"
#include "software/world/world.h"

/**
 * The NavigatingPrimitiveCreator converts the given NavigatingIntents into their
 * respective Primitives given the associated path
 */
class NavigatingPrimitiveCreator : public NavigatingIntentVisitor
{
   public:
    /**
     * Creates a primitive for a given path and navigating intent
     *
     * @param intent The NavigatingIntent to make primitive from
     * @param path path to make primitive for
     * @param enemy_robot_obstacles list of enemy robot obstacles to watch out for
     *
     * @return Primitive
     */
    TbotsProto::Primitive createNavigatingPrimitive(
        const NavigatingIntent &intent, const Path &path,
        const std::vector<ObstaclePtr> &enemy_robot_obstacles);

    /**
     * Converts the given NavigatingIntent into a Primitive
     *
     * @param The NavigatingIntent to convert
     */
    void visit(const MoveIntent &intent) override;

    /**
     * Calculates the transition speed for the robot between two line segments
     *
     * Calculates the speed that the robot should be at when it is at the end of a
     * given line segment in order to smoothly transition to another given line segment,
     * given a final speed at the end of the two line segments
     *
     * This is only public so it is testable.
     *
     * @param p1, p2, p3 are 3 points that define two line segments that form a path
     * @param final_speed is the intended final speed at the end of the path
     * @return the first segment's final speed after travelling from p1 to p2
     * for a smooth transition to the p2 to p3 path, scaled by the final speed at the end
     * of the path
     */
    static double calculateTransitionSpeedBetweenSegments(const Point &p1,
                                                          const Point &p2,
                                                          const Point &p3,
                                                          double final_speed);

   private:
    /**
     * Calculates a factor for how close p is to an enemy obstacle.
     * 0 = touching or inside
     * 1 = greater than/equal to EnemyRobotProximityLimit (dynamic parameter) away
     * scaled linearly between these values
     *
     * @param p point to evaluate
     * @param enemy_robot_obstacles list of enemy robot obstacles to watch out for
     *
     * @return A factor from 0 to 1 for how close p is to an enemy obstacle
     */
    double getEnemyObstacleProximityFactor(
        const Point &p, const std::vector<ObstaclePtr> &enemy_robot_obstacles) const;

    /**
     * Creates the final speed and destination given the final speed and the path
     *
     * @param final_speed The final speed
     * @param path path to make primitive for
     * @param enemy_robot_obstacles list of enemy robot obstacles to watch out for
     * @param robot_constants The robot constants
     *
     * @return the final destination and speed
     */
    std::pair<Point, double> calculateDestinationAndFinalSpeed(
        double final_speed, Path path,
        const std::vector<ObstaclePtr> &enemy_robot_obstacles,
        const RobotConstants_t &robot_constants) const;

    std::optional<TbotsProto::Primitive> current_primitive;
    Point new_destination;
    double new_final_speed;
};
