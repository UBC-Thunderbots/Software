#pragma once


#include "software/ai/hl/stp/tactic/primitive.h"
#include "software/ai/navigator/path_planner/bang_bang_trajectory_2d.h"
#include "software/ai/navigator/path_planner/bang_bang_trajectory_1d_angular.h"
#include "software/world/world.h"
#include "proto/primitive/primitive_types.h"
#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"
#include "software/ai/navigator/path_planner/trajectory_planner.h"

// Forward declare Tactic to avoid circular dependency between Tactic and MovePrimitive
class Tactic;

class MovePrimitive : public Primitive
{
public:
    MovePrimitive(
            const World &world,
            const shared_ptr<Tactic> &tactic,
            const Robot &robot,
            const TbotsProto::RobotNavigationObstacleConfig &config,
            const Point &destination,
            const Angle &final_angle,
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
    std::unique_ptr<TbotsProto::Primitive> generatePrimitiveProtoMessage() override;

private:
    std::vector<ObstaclePtr> generateObstacles() const;

    World world;
    std::shared_ptr<Tactic> tactic;
    Robot robot;
    Point destination;
    Angle final_angle;

    TbotsProto::DribblerMode dribbler_mode;
    AutoChipOrKick auto_chip_or_kick;

    TbotsProto::BallCollisionType ball_collision_type;
    TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode;

    BangBangTrajectory2D trajectory;
    BangBangTrajectory1DAngular angular_trajectory;

    RobotNavigationObstacleFactory obstacle_factory;
    TrajectoryPlanner planner;
};
