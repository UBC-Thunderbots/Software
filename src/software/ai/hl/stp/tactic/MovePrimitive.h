#pragma once


#include "software/ai/hl/stp/tactic/primitive.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/navigator/path_planner/bang_bang_trajectory_2d.h"
#include "software/ai/navigator/path_planner/bang_bang_trajectory_1d_angular.h"
#include "software/world/world.h"
#include "software/world/robot.h"
#include "proto/primitive/primitive_types.h"

class MovePrimitive : public Primitive
{
public:
    MovePrimitive(
            const World &world,
            const Tactic &tactic,
            const Robot &robot, const Point &destination,
            const TbotsProto::MaxAllowedSpeedMode &max_allowed_speed_mode, const Angle &final_angle,
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
    TbotsProto::Primitive generatePrimitiveProtoMessage() const override;

private:
    World world;
    std::shared_ptr<Tactic> tactic;
    Robot robot;
    Point destination;
    Angle final_angle;

    TbotsProto::DribblerMode dribbler_mode;
    AutoChipOrKick auto_chip_or_kick;

    TbotsProto::BallCollisionType ball_collision_type;
    TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode;
    RobotConstants_t robot_constants;

    BangBangTrajectory2D trajectory;
    BangBangTrajectory1DAngular angular_trajectory;
};
