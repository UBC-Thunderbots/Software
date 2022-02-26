#include "software/ai/hl/stp/tactic/tactic_factory.h"

#include "proto/message_translation/tbots_geometry.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "software/logger/logger.h"

std::shared_ptr<Tactic> createTactic(const TbotsProto::AttackerTactic &tactic_proto)
{
    auto config = std::make_shared<AttackerTacticConfig>();
    config->loadFromProto(tactic_proto.attacker_tactic_config());
    auto tactic = std::make_shared<AttackerTactic>(config);

    if (tactic_proto.has_best_pass_so_far())
    {
        tactic->updateControlParams(createPass(tactic_proto.best_pass_so_far()),
                                    tactic_proto.pass_committed());
    }
    if (tactic_proto.has_chip_target())
    {
        tactic->updateControlParams(createPoint(tactic_proto.chip_target()));
    }

    return tactic;
}

std::shared_ptr<Tactic> createTactic(const TbotsProto::ChipTactic &tactic_proto)
{
    auto tactic = std::make_shared<ChipTactic>();
    tactic->updateControlParams(createPoint(tactic_proto.chip_origin()),
                                createAngle(tactic_proto.chip_direction()),
                                tactic_proto.chip_distance_meters());
    return tactic;
}

std::shared_ptr<Tactic> createTactic(const TbotsProto::CreaseDefenderTactic &tactic_proto)
{
    auto config = std::make_shared<RobotNavigationObstacleConfig>();

    config->loadFromProto(tactic_proto.robot_navigation_obstacle_config());
    auto tactic = std::make_shared<CreaseDefenderTactic>(config);

    tactic->updateControlParams(createPoint(tactic_proto.enemy_threat_origin()),
                                tactic_proto.crease_defender_alignment(),
                                tactic_proto.max_allowed_speed_mode());

    return tactic;
}

std::shared_ptr<Tactic> createTactic(const TbotsProto::DribbleTactic &tactic_proto)
{
    auto tactic                                    = std::make_shared<DribbleTactic>();
    std::optional<Point> dribble_destination       = std::nullopt;
    std::optional<Angle> final_dribble_orientation = std::nullopt;
    if (tactic_proto.has_dribble_destination())
    {
        dribble_destination = createPoint(tactic_proto.dribble_destination());
    }
    if (tactic_proto.has_final_dribble_orientation())
    {
        final_dribble_orientation = createAngle(tactic_proto.final_dribble_orientation());
    }

    tactic->updateControlParams(dribble_destination, final_dribble_orientation,
                                tactic_proto.allow_excessive_dribbling());
    return tactic;
}

std::shared_ptr<Tactic> createTactic(const TbotsProto::GetBehindBallTactic &tactic_proto)
{
    auto tactic = std::make_shared<GetBehindBallTactic>();
    tactic->updateControlParams(createPoint(tactic_proto.ball_location()),
                                createAngle(tactic_proto.chick_direction()));
    return tactic;
}

std::shared_ptr<Tactic> createTactic(const TbotsProto::GoalieTactic &tactic_proto)
{
    auto config = std::shared_ptr<GoalieTacticConfig>();
    config->loadFromProto(tactic_proto.goalie_tactic_config());

    auto tactic =
        std::make_shared<GoalieTactic>(config, tactic_proto.max_allowed_speed_mode());
    return tactic;
}

std::shared_ptr<Tactic> createTactic(const TbotsProto::KickTactic &tactic_proto)
{
    auto tactic = std::make_shared<KickTactic>();
    tactic->updateControlParams(createPoint(tactic_proto.kick_origin()),
                                createAngle(tactic_proto.kick_direction()),
                                tactic_proto.kick_speed_meters_per_second());
    return tactic;
}

std::shared_ptr<Tactic> createTactic(
    const TbotsProto::MoveGoalieToGoalLineTactic &tactic_proto)
{
    auto tactic = std::make_shared<MoveGoalieToGoalLineTactic>();
    return tactic;
}

std::shared_ptr<Tactic> createTactic(const TbotsProto::MoveTactic &tactic_proto)
{
    auto tactic = std::make_shared<MoveTactic>();
    tactic->updateControlParams(
        createPoint(tactic_proto.destination()),
        createAngle(tactic_proto.final_orientation()), tactic_proto.final_speed(),
        tactic_proto.dribbler_mode(), tactic_proto.ball_collision_type(),
        createAutoChipOrKick(tactic_proto.auto_chip_or_kick()),
        tactic_proto.max_allowed_speed_mode(), tactic_proto.target_spin_rev_per_s());
    return tactic;
}

std::shared_ptr<Tactic> createTactic(const TbotsProto::PenaltyKickTactic &tactic_proto)
{
    auto tactic = std::make_shared<PenaltyKickTactic>();
    return tactic;
}

std::shared_ptr<Tactic> createTactic(const TbotsProto::PivotKickTactic &tactic_proto)
{
    auto tactic = std::make_shared<PivotKickTactic>();
    tactic->updateControlParams(createPoint(tactic_proto.kick_origin()),
                                createAngle(tactic_proto.kick_direction()),
                                createAutoChipOrKick(tactic_proto.auto_chip_or_kick()));
    return tactic;
}

std::shared_ptr<Tactic> createTactic(const TbotsProto::ReceiverTactic &tactic_proto)
{
    auto tactic              = std::make_shared<ReceiverTactic>();
    std::optional<Pass> pass = std::nullopt;
    if (tactic_proto.has_pass())
    {
        pass = createPass(tactic_proto.pass());
    }

    tactic->updateControlParams(pass, tactic_proto.disable_one_touch_shot());
    return tactic;
}

std::shared_ptr<Tactic> createTactic(const TbotsProto::ShadowEnemyTactic &tactic_proto)
{
    auto tactic                             = std::make_shared<ShadowEnemyTactic>();
    std::optional<EnemyThreat> enemy_threat = std::nullopt;
    if (tactic_proto.has_enemy_threat())
    {
        enemy_threat = createEnemyThreat(tactic_proto.enemy_threat());
    }

    tactic->updateControlParams(enemy_threat, tactic_proto.shadow_distance());
    return tactic;
}

std::shared_ptr<Tactic> createTactic(const TbotsProto::StopTactic &tactic_proto)
{
    auto tactic = std::make_shared<StopTactic>(tactic_proto.coast());
    return tactic;
}

AutoChipOrKick createAutoChipOrKick(
    const TbotsProto::AutoChipOrKick &auto_chip_or_kick_proto)
{
    AutoChipOrKick auto_chip_or_kick = {AutoChipOrKickMode::OFF, 0};
    if (auto_chip_or_kick_proto.has_autochip_distance_meters())
    {
        auto_chip_or_kick = {AutoChipOrKickMode::AUTOCHIP,
                             auto_chip_or_kick_proto.autochip_distance_meters()};
    }
    if (auto_chip_or_kick_proto.has_autokick_speed_m_per_s())
    {
        auto_chip_or_kick = {AutoChipOrKickMode::AUTOKICK,
                             auto_chip_or_kick_proto.autokick_speed_m_per_s()};
    }
    return auto_chip_or_kick;
}

Pass createPass(const TbotsProto::Pass &pass_proto)
{
    return Pass(createPoint(pass_proto.passer_point()),
                createPoint(pass_proto.receiver_point()),
                pass_proto.pass_speed_m_per_s());
}

EnemyThreat createEnemyThreat(const TbotsProto::EnemyThreat &enemy_threat_proto)
{
    std::optional<Angle> best_shot_angle;
    if (enemy_threat_proto.has_best_shot_angle())
    {
        best_shot_angle = createAngle(enemy_threat_proto.best_shot_angle());
    }

    std::optional<Point> best_shot_target;
    if (enemy_threat_proto.has_best_shot_target())
    {
        best_shot_target = createPoint(enemy_threat_proto.best_shot_target());
    }

    return EnemyThreat{Robot(enemy_threat_proto.robot()),
                       enemy_threat_proto.has_ball(),
                       createAngle(enemy_threat_proto.goal_angle()),
                       best_shot_angle,
                       best_shot_target,
                       enemy_threat_proto.num_passes_to_get_possession(),
                       Robot(enemy_threat_proto.passer())};
}
