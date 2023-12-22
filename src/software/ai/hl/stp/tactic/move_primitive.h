#pragma once


#include "proto/primitive/primitive_types.h"
#include "software/ai/hl/stp/tactic/primitive.h"
#include "software/ai/navigator/path_planner/bang_bang_trajectory_1d_angular.h"
#include "software/ai/navigator/path_planner/bang_bang_trajectory_2d.h"
#include "software/ai/navigator/path_planner/trajectory_planner.h"
#include "software/world/world.h"

// Forward declare Tactic to avoid circular dependency between Tactic and MovePrimitive
class Tactic;

class MovePrimitive : public Primitive
{
   public:
    /**
     * Create a Move Primitive Message TODO: Double check docs
     *
     * @param dest The final destination of the movement
     * @param final_speed_m_per_s The speed at final destination
     * @param final_angle The final orientation the robot should have at the end
     * of the movement
     * @param should_drive_forward Whether the robot should face the direction of
     * intermediate path points (if there is any) or just the final destination
     * @param dribbler_mode The dribbler mode
     * @param auto_chip_or_kick The command to autochip or autokick
     * @param max_allowed_speed_mode The mode of maximum speed allowed
     * @param target_spin_rev_per_s The target spin while moving in revolutions per second
     * @param robot_constants The robot constants
     * @param cost_override optionally override the cost of the move primitive, defaults
     * to the path length
     */
    MovePrimitive(const Robot &robot, const Point &destination, const Angle &final_angle,
                  const TbotsProto::MaxAllowedSpeedMode &max_allowed_speed_mode,
                  const TbotsProto::DribblerMode &dribbler_mode,
                  const TbotsProto::BallCollisionType &ball_collision_type,
                  const AutoChipOrKick &auto_chip_or_kick,
                  std::optional<double> cost_override = std::nullopt);

    ~MovePrimitive() override = default;

    /**
     * Gets the primitive proto message
     *
     * @return the primitive proto message
     */
    std::unique_ptr<TbotsProto::Primitive> generatePrimitiveProtoMessage(
        const World &world,
        const std::set<TbotsProto::MotionConstraint> &motion_constraints,
        const RobotNavigationObstacleFactory &obstacle_factory) override;

    /**
     * Get the obstacles generated so far
     *
     * @return the obstacles generated so far
     */
    std::vector<ObstaclePtr> getGeneratedObstacles() const override;

   private:
    /**
     * Fills the `obstacles` vector with the obstacles that the primitive should avoid
     *
     * @param world TODO (NIMA)
     * @param motion_constraints
     * @param obstacle_factory
     */
    void generateObstacles(
        const World &world,
        const std::set<TbotsProto::MotionConstraint> &motion_constraints,
        const RobotNavigationObstacleFactory &obstacle_factory);

    Robot robot;
    Point destination;
    Angle final_angle;

    TbotsProto::DribblerMode dribbler_mode;
    AutoChipOrKick auto_chip_or_kick;

    TbotsProto::BallCollisionType ball_collision_type;
    TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode;

    std::vector<ObstaclePtr> obstacles;

    BangBangTrajectory2D trajectory;
    BangBangTrajectory1DAngular angular_trajectory;
    TrajectoryPlanner planner;
};
