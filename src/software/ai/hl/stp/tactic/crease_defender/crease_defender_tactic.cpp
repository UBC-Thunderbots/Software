#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/geom/algorithms/intersection.h"
#include "software/geom/point.h"
#include "software/geom/ray.h"
#include "software/geom/segment.h"
#include "software/logger/logger.h"

CreaseDefenderTactic::CreaseDefenderTactic(TbotsProto::AiConfig ai_config)
    : Tactic({RobotCapability::Move}),
      fsm_map(),
      control_params({Point(0, 0), std::nullopt,
                      TbotsProto::CreaseDefenderAlignment::CENTRE,
                      TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT}),
      ai_config(ai_config)
{
    for (RobotId id = 0; id < MAX_ROBOT_IDS; id++)
    {
        fsm_map[id] = std::make_unique<FSM<CreaseDefenderFSM>>(
            CreaseDefenderFSM(ai_config.robot_navigation_obstacle_config()),
            DribbleFSM(ai_config.dribble_tactic_config()));
    }
}

void CreaseDefenderTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}

void CreaseDefenderTactic::updateControlParams(
    const Point &enemy_threat_origin, const std::optional<Point> &block_threat_point,
    const TbotsProto::CreaseDefenderAlignment &alignment,
    TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode)
{
    control_params.enemy_threat_origin       = enemy_threat_origin;
    control_params.block_threat_point        = block_threat_point;
    control_params.crease_defender_alignment = alignment;
    control_params.max_allowed_speed_mode    = max_allowed_speed_mode;
}

void CreaseDefenderTactic::updatePrimitive(const TacticUpdate &tactic_update,
                                           bool reset_fsm)
{
    if (reset_fsm)
    {
        fsm_map[tactic_update.robot.id()] = std::make_unique<FSM<CreaseDefenderFSM>>(
            CreaseDefenderFSM(ai_config.robot_navigation_obstacle_config()));
        //            DribbleFSM(ai_config.dribble_tactic_config()));
    }
    //    Point ball_position       = tactic_update.world_ptr->ball().position();
    //    Point enemy_goal_center   =
    //    tactic_update.world_ptr->field().enemyGoal().centre(); Vector ball_to_net_vector
    //    = Vector(enemy_goal_center.x() - ball_position.x(),
    //                                       enemy_goal_center.y() - ball_position.y());
    //    DribbleFSM::ControlParams dribble_control_params{
    //        .dribble_destination       = ball_position,
    //        .final_dribble_orientation = ball_to_net_vector.orientation(),
    //        .allow_excessive_dribbling = false};

    fsm_map.at(tactic_update.robot.id())
        ->process_event(CreaseDefenderFSM::Update(control_params, tactic_update));
    //    fsm_map.at(tactic_update.robot.id())
    //        ->process_event(DribbleFSM::Update(dribble_control_params, tactic_update));
}
